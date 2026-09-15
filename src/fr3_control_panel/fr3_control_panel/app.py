"""PyQt5 UI. --demo previews widgets without importing ROS or connecting hardware."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from pathlib import Path
import sys
import time
import math
import yaml
from PyQt5 import QtCore, QtWidgets
from .core import save_points, load_points


class Signals(QtCore.QObject):
    finished = QtCore.pyqtSignal(str)
    failed = QtCore.pyqtSignal(str)
    log = QtCore.pyqtSignal(str)


class Panel(QtWidgets.QMainWindow):
    def __init__(self, cfg, backend=None, session=None, robot_ip='192.168.58.2', serial_port='/dev/ttyACM0'):
        super().__init__()
        self.cfg, self.backend = cfg, backend
        self.session = session
        self.disconnecting = False
        self.pending_points = None
        self.grasp_ready = False
        self.points = {}
        self.busy = False
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.signals = Signals()
        self.signals.finished.connect(self.done)
        self.signals.failed.connect(lambda s: self.done('失败：'+s))
        self.signals.log.connect(self.write_log)
        self.setWindowTitle('FR3 · 机械臂与夹爪控制台')
        self.resize(1180, 820)
        body = QtWidgets.QWidget()
        self.setCentralWidget(body)
        layout = QtWidgets.QVBoxLayout(body)
        title = QtWidgets.QLabel('单机械臂 · 示教与抓取')
        title.setStyleSheet('font-size:25px; font-weight:600')
        layout.addWidget(title)
        self.status = QtWidgets.QLabel('离线界面预览 — 无设备连接' if backend is None else '未连接 — 请点击连接')
        layout.addWidget(self.status)
        connection_row = QtWidgets.QHBoxLayout()
        layout.addLayout(connection_row)
        connection_row.addWidget(QtWidgets.QLabel('机械臂 IP'))
        self.ip_input = QtWidgets.QLineEdit(robot_ip)
        self.ip_input.setMaximumWidth(150)
        connection_row.addWidget(self.ip_input)
        connection_row.addWidget(QtWidgets.QLabel('夹爪串口'))
        self.serial_input = QtWidgets.QLineEdit(serial_port)
        self.serial_input.setMaximumWidth(200)
        connection_row.addWidget(self.serial_input)
        self.connect_button = self.button(connection_row, '连接机械臂与夹爪', self.connect_devices,
                                           tracked=False)
        self.disconnect_button = self.button(connection_row, '断开连接', self.disconnect_devices,
                                              tracked=False)
        self.disconnect_button.setEnabled(False)
        self.real_confirm = QtWidgets.QCheckBox('现场已确认可启动驱动（连接可能发送保持指令）')
        self.real_confirm.setVisible(session is not None and not backend.mock)
        layout.addWidget(self.real_confirm)
        self.arm_motion = QtWidgets.QCheckBox('允许运动 / 夹爪操作（默认关闭；取消勾选将请求停止当前动作）')
        self.arm_motion.toggled.connect(self.set_armed)
        layout.addWidget(self.arm_motion)
        layout.addWidget(QtWidgets.QLabel('TCP 由实测关节 + 已配置工具模型计算；碰撞检测只覆盖模型和已添加障碍物。'))
        layout.addWidget(QtWidgets.QLabel(f"参考坐标系：{cfg['frame']}    TCP：{cfg['tcp']}    位姿单位：m / deg"))
        columns = QtWidgets.QHBoxLayout()
        layout.addLayout(columns)
        left = QtWidgets.QVBoxLayout()
        right = QtWidgets.QVBoxLayout()
        columns.addLayout(left, 1)
        columns.addLayout(right, 2)
        feedback = QtWidgets.QGroupBox('实时状态')
        form = QtWidgets.QFormLayout(feedback)
        self.joint_labels = []
        for name in cfg['joints']:
            label = QtWidgets.QLabel('— °')
            self.joint_labels.append(label)
            form.addRow(name, label)
        self.open_label = QtWidgets.QLabel('— %')
        form.addRow('夹爪开度', self.open_label)
        self.tcp_label = QtWidgets.QLabel('—')
        self.tcp_label.setWordWrap(True)
        form.addRow('TCP', self.tcp_label)
        left.addWidget(feedback)
        gripbox = QtWidgets.QGroupBox('夹爪控制 · 0% 闭合 / 100% 张开')
        gf = QtWidgets.QVBoxLayout(gripbox)
        self.opening = QtWidgets.QSpinBox()
        self.opening.setRange(0, 100)
        self.opening.setValue(100)
        self.opening.setSuffix(' %')
        gf.addWidget(self.opening)
        self.command_buttons = []
        self.motion_buttons = []
        self.button(gf, '设置夹爪开度', self.set_gripper, motion=True)
        self.button(gf, '释放物体 / 张开', self.release, motion=True)
        left.addWidget(gripbox)
        self.button(left, '取消当前操作', self.cancel, tracked=False)
        left.addWidget(QtWidgets.QLabel('软件取消依赖控制器响应，不等同于硬件急停。'))
        left.addStretch()

        target = QtWidgets.QGroupBox('笛卡尔目标位姿')
        grid = QtWidgets.QGridLayout(target)
        self.fields = []
        for i, name in enumerate(['X (m)', 'Y (m)', 'Z (m)', 'Rx (°)', 'Ry (°)', 'Rz (°)']):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-10 if i < 3 else -360, 10 if i < 3 else 360)
            spin.setDecimals(4 if i < 3 else 2)
            spin.setSingleStep(.01 if i < 3 else 1)
            self.fields.append(spin)
            grid.addWidget(QtWidgets.QLabel(name), i//3*2, i%3)
            grid.addWidget(spin, i//3*2+1, i%3)
        row = QtWidgets.QHBoxLayout()
        grid.addLayout(row, 4, 0, 1, 3)
        self.button(row, '当前 TCP 填入', self.fill_current)
        self.button(row, '仅规划 / RViz 预览', lambda: self.move_target(False))
        self.button(row, '规划并到达', lambda: self.move_target(True), motion=True)
        right.addWidget(target)
        teach = QtWidgets.QGroupBox('手动点位采集与流程')
        tv = QtWidgets.QVBoxLayout(teach)
        row = QtWidgets.QHBoxLayout()
        tv.addLayout(row)
        self.button(row, '采集抓取位姿', lambda: self.capture('抓取'))
        self.button(row, '采集展示位姿', lambda: self.capture('展示'))
        self.button(row, '采集途经点', self.capture_waypoint)
        self.list = QtWidgets.QListWidget()
        self.list.setMaximumHeight(145)
        self.list.itemDoubleClicked.connect(self.load_selected)
        tv.addWidget(self.list)
        row = QtWidgets.QHBoxLayout()
        tv.addLayout(row)
        self.button(row, '目标框保存为选中点', self.replace_selected)
        self.button(row, '删除选中点', self.delete_selected)
        self.button(row, '上移', lambda: self.reorder_selected(-1))
        self.button(row, '下移', lambda: self.reorder_selected(1))
        self.button(row, '保存 JSON', self.save)
        self.button(row, '载入 JSON', self.load)
        self.button(tv, '① 到抓取点并闭合夹爪', self.pick, motion=True)
        self.confirm_grasp_button = self.button(tv, '② 确认夹持和负载模型 → 途经点 → 展示', self.confirm_grasp, motion=True)
        tv.addWidget(QtWidgets.QLabel('闭合后暂停，人工确认夹持再搬运；展示点保持夹持，不自动松爪。'))
        tv.addWidget(QtWidgets.QLabel('双击点位填入目标框；途经点按列表顺序搬运，可上移/下移。'))
        right.addWidget(teach)
        obs = QtWidgets.QGroupBox('添加 / 更新盒形障碍物')
        ov = QtWidgets.QVBoxLayout(obs)
        self.obstacle_name = QtWidgets.QLineEdit('fixture_1')
        ov.addWidget(self.obstacle_name)
        self.size_fields = []
        row = QtWidgets.QHBoxLayout()
        ov.addLayout(row)
        for name in ['长', '宽', '高']:
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(.001, 10)
            s.setDecimals(3)
            s.setValue(.1)
            s.setPrefix(name+' ')
            s.setSuffix(' m')
            self.size_fields.append(s)
            row.addWidget(s)
        self.button(ov, '使用目标框位姿添加障碍物', self.add_obstacle)
        right.addWidget(obs)
        self.logs = QtWidgets.QPlainTextEdit()
        self.logs.setReadOnly(True)
        self.logs.setMaximumBlockCount(300)
        self.logs.setMaximumHeight(120)
        layout.addWidget(self.logs)
        self.setStyleSheet('QGroupBox {font-weight:600; margin-top:12px; padding-top:12px;} '
                           'QPushButton {padding:7px;} QDoubleSpinBox,QSpinBox {padding:5px;}')
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(150)
        if backend is None:
            self.write_log('离线预览：所有运动与采集按钮禁用。请在 Ubuntu / ROS 2 中启动实际后端。')
        self.refresh()

    def button(self, layout, text, callback, tracked=True, motion=False):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(lambda _=False: self.guard(callback))
        layout.addWidget(button)
        if tracked:
            self.command_buttons.append(button)
        if motion:
            self.motion_buttons.append(button)
        return button

    def guard(self, callback):
        try:
            callback()
        except Exception as exc:
            self.write_log('失败：'+str(exc))

    def write_log(self, text):
        self.logs.appendPlainText(time.strftime('%H:%M:%S')+'  '+text)

    def target(self):
        return [s.value() for s in self.fields]

    def fill(self, values):
        for s, value in zip(self.fields, values):
            s.setValue(value)

    def fill_current(self):
        self.fill(self.backend.capture())

    def capture(self, name):
        self.points[name] = self.backend.capture()
        self.update_points()
        self.write_log('已采集 '+name)

    def capture_waypoint(self):
        i = 1
        while f'途经{i}' in self.points:
            i += 1
        self.capture(f'途经{i}')

    def update_points(self):
        self.list.clear()
        for name, v in self.points.items():
            item = QtWidgets.QListWidgetItem(name+'  '+', '.join(f'{x:.3f}' for x in v))
            item.setData(QtCore.Qt.UserRole, name)
            self.list.addItem(item)

    def selected(self):
        item = self.list.currentItem()
        if item is None:
            raise ValueError('请先选择点位')
        return item.data(QtCore.Qt.UserRole)

    def load_selected(self, item):
        self.fill(self.points[item.data(QtCore.Qt.UserRole)])

    def replace_selected(self):
        self.points[self.selected()] = self.target()
        self.update_points()

    def delete_selected(self):
        del self.points[self.selected()]
        self.update_points()

    def reorder_selected(self, offset):
        name = self.selected()
        keys = list(self.points)
        index = keys.index(name)
        target = index + offset
        if 0 <= target < len(keys):
            keys[index], keys[target] = keys[target], keys[index]
            self.points = {key: self.points[key] for key in keys}
            self.update_points()
            self.list.setCurrentRow(target)

    def set_armed(self, armed):
        if self.backend:
            self.backend.motion_armed = armed
            if not armed:
                self.backend.cancel()
        self.refresh()

    def save(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, '保存点位', 'points.json', '*.json')
        if path:
            save_points(path, self.cfg['frame'], self.cfg['tcp'], self.points)
            self.write_log('点位已保存：'+path)

    def load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '载入点位', '', '*.json')
        if path:
            self.points = load_points(path, self.cfg['frame'], self.cfg['tcp'])
            self.update_points()

    def run(self, name, function):
        if self.busy:
            raise RuntimeError('请等待当前操作结束')
        if self.backend is None:
            raise RuntimeError('离线预览不能控制设备')
        self.backend.begin()
        self.busy = True
        self.write_log(name+'…')
        self.refresh()
        def work():
            try:
                function()
                self.signals.finished.emit(name+'完成')
            except Exception as exc:
                self.signals.failed.emit(str(exc))
        self.pool.submit(work)

    def connect_devices(self):
        if self.backend is None:
            raise RuntimeError('离线预览不能连接设备')
        if self.busy:
            raise RuntimeError('请等待当前操作结束')
        self.backend.prepare_connection()
        self.busy = True
        ip, serial = self.ip_input.text().strip(), self.serial_input.text().strip()
        confirmed = self.real_confirm.isChecked()
        self.disconnecting = False
        self.status.setText('连接检查中…')
        self.connect_button.setEnabled(False)
        def work():
            try:
                if self.session:
                    self.backend.check_namespace_free()
                    self.session.start(ip, serial, confirmed)
                    self.signals.log.emit('后端日志：'+self.session.log_path)
                    self.backend.connect(timeout=75, health_check=self.session.check)
                else:
                    self.backend.connect()
                self.signals.finished.emit('机械臂与夹爪已连接')
            except Exception as exc:
                if self.session:
                    try:
                        self.session.stop()
                    except Exception as stop_exc:
                        self.backend.fault = str(stop_exc)
                self.signals.failed.emit('连接失败：'+str(exc))
        self.pool.submit(work)
        self.refresh()

    def disconnect_devices(self):
        if self.backend is not None and not self.disconnecting:
            self.disconnecting = True
            self.backend.disconnect()
            self.arm_motion.setChecked(False)
            self.busy = True
            self.write_log('已请求取消；等待当前操作结束并断开面板连接…')
            # Same single worker: do not re-enable commands while old work unwinds.
            def cleanup():
                try:
                    self.backend.disconnect()
                    if self.session:
                        self.session.stop()
                    self.signals.finished.emit('断开完成')
                except Exception as exc:
                    self.backend.fault = str(exc)
                    self.signals.failed.emit(str(exc))
            self.pool.submit(cleanup)
            self.refresh()

    def done(self, text):
        if self.disconnecting and text != '断开完成' and not getattr(self.backend, 'fault', None):
            self.write_log(text)
            return
        self.disconnecting = False
        self.busy = False
        self.write_log(text)
        self.refresh()

    def move_target(self, execute):
        values = self.target()
        self.run('规划并到达' if execute else '规划预览',
                 lambda: self.backend.motion(values, execute))

    def set_gripper(self):
        opening = self.opening.value()
        self.run('调整夹爪', lambda: self.backend.gripper(opening))

    def pick(self):
        if '抓取' not in self.points or '展示' not in self.points:
            raise ValueError('请先采集抓取位姿和展示位姿')
        points = deepcopy(self.points)
        self.pending_points = points
        self.grasp_ready = False
        def sequence():
            if self.backend.payload or getattr(self.backend, 'grasp_pending', False):
                raise RuntimeError('已有挂载物体或未确认夹持，请先释放')
            self.backend.gripper(100)
            self.signals.log.emit('规划至抓取点')
            self.backend.motion(points['抓取'], True)
            self.signals.log.emit('夹取并检查夹爪反馈')
            self.backend.gripper(0, grasp=True)
            self.grasp_ready = True
            self.signals.log.emit('已暂停：停滞不代表可靠夹持。请现场核对物体、夹持和配置的负载包围盒，再点击②。')
        self.run('到抓取点并闭合', sequence)

    def confirm_grasp(self):
        if not self.grasp_ready or self.pending_points is None:
            raise RuntimeError('请先完成步骤①并检查夹持状态')
        points = deepcopy(self.pending_points)
        self.grasp_ready = False
        def sequence():
            self.backend.attach()
            for name, values in points.items():
                if name.startswith('途经'):
                    self.signals.log.emit('搬运至 '+name)
                    self.backend.motion(values, True)
            self.signals.log.emit('搬运至展示点')
            self.backend.motion(points['展示'], True)
        self.run('确认夹持后搬运至展示点', sequence)

    def release(self):
        self.grasp_ready = False
        self.pending_points = None
        self.run('释放物体', self.backend.release)

    def add_obstacle(self):
        name, values = self.obstacle_name.text(), self.target()
        sizes = [s.value() for s in self.size_fields]
        self.run('更新碰撞场景', lambda: self.backend.obstacle(name, values, sizes))

    def cancel(self):
        self.grasp_ready = False
        if self.backend:
            self.backend.cancel()
            self.write_log('已请求取消当前动作与后续流程')

    def refresh(self):
        if self.session and self.session.process is not None and self.backend.connected:
            if self.session.process.poll() is not None and not self.backend.fault:
                self.backend.fault = '后端进程已退出，请断开并检查日志：'+str(self.session.log_path)
                self.backend.cancel()
        connected = self.backend is not None and getattr(self.backend, 'connected', False)
        fault = getattr(self.backend, 'fault', None)
        editable = self.session is not None and not self.busy and not connected
        self.ip_input.setEnabled(editable)
        self.serial_input.setEnabled(editable)
        self.real_confirm.setEnabled(editable)
        self.arm_motion.setEnabled(connected and not fault and getattr(self.backend, 'execution_enabled', False))
        if self.backend is not None:
            connected = getattr(self.backend, 'connected', False)
            fault = getattr(self.backend, 'fault', None)
            self.connect_button.setEnabled(not self.busy and not connected and not fault)
            self.disconnect_button.setEnabled((connected or self.busy) and not self.disconnecting)
        for b in self.command_buttons:
            b.setEnabled(bool(connected and not self.busy and not fault))
        for b in self.motion_buttons:
            b.setEnabled(b.isEnabled() and self.arm_motion.isChecked() and getattr(self.backend, 'execution_enabled', False))
        self.confirm_grasp_button.setEnabled(self.confirm_grasp_button.isEnabled() and self.grasp_ready)
        if self.backend is None:
            return
        values, pose, stamp = self.backend.snapshot()
        if not connected:
            values, pose, stamp = {}, None, 0.0
        now = time.monotonic()
        timeout = self.cfg['feedback_timeout']
        for name, label in zip(self.cfg['joints'], self.joint_labels):
            v = values.get(name)
            label.setText(f'{math.degrees(v[0]):.2f} °' if v and now-v[1] < timeout else '— 反馈过期')
        g = values.get(self.cfg['gripper_joint'])
        self.open_label.setText(f'{100*(1-g[0]/self.cfg["closed_position"]):.1f} %'
                                if g and now-g[1] < timeout else '— 反馈过期')
        self.tcp_label.setText('\n'.join([', '.join(f'{v:.4f}' for v in pose[:3])+' m',
                                        ', '.join(f'{v:.2f}' for v in pose[3:])+' °'])
                               if pose and now-stamp < timeout else '— TCP 反馈过期')
        connected = getattr(self.backend, 'connected', False)
        self.status.setText(('操作中' if self.busy else ('已连接 / 就绪' if connected else '未连接'))+
                            (' · MOCK 虚拟设备' if self.backend.mock else ' · ROS 设备连接模式')+
                            (' · 已挂载物体' if self.backend.payload else ''))
        if connected and not self.arm_motion.isChecked():
            self.status.setText(self.status.text()+' · 仅观察 / 规划，运动未授权')
        if self.grasp_ready:
            self.status.setText(self.status.text()+' · 等待人工确认夹持')
        if getattr(self.backend, 'fault', None):
            self.status.setText(self.backend.fault)

    def closeEvent(self, event):
        self.cancel()
        if self.busy:
            self.write_log('等待后台操作退出后再次关闭窗口')
            event.ignore()
            return
        if self.session and self.session.process is not None:
            self.disconnect_devices()
            self.write_log('正在关闭后端，断开完成后可关闭窗口')
            event.ignore()
            return
        self.pool.shutdown(wait=False)
        event.accept()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true', help='UI only, no ROS imports')
    parser.add_argument('--mock', action='store_true', help='Only with mock bringup: allow virtual empty grasp')
    parser.add_argument('--config')
    parser.add_argument('--allow-execution', action='store_true', help='Allow UI arming; defaults to observation/planning')
    parser.add_argument('--managed', action='store_true', help='Start single-arm bringup when Connect is clicked')
    parser.add_argument('--robot-ip', default='192.168.58.2')
    parser.add_argument('--serial-port', default='/dev/ttyACM0')
    parser.add_argument('--real-config', default='')
    parser.add_argument('--cell', default='')
    # launch_ros appends ROS-specific arguments such as --ros-args.
    args, ros_args = parser.parse_known_args()
    if ros_args and ros_args[0] != '--ros-args':
        parser.error('未知参数：'+' '.join(ros_args))
    if args.config:
        path = Path(args.config)
    elif args.demo:
        path = Path(__file__).resolve().parents[1]/'config/panel.yaml'
        if not path.is_file():
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory('fr3_control_panel'))/'config/panel.yaml'
    else:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('fr3_control_panel'))/'config/panel.yaml'
    cfg = yaml.safe_load(path.read_text(encoding='utf-8'))
    from .validation import validate_config
    validate_config(cfg)
    app = QtWidgets.QApplication(sys.argv[:1])
    backend = None
    if not args.demo:
        import rclpy
        from .backend import Backend
        rclpy.init(args=ros_args)
        backend = Backend(cfg, args.mock, args.allow_execution)
    session = None
    if args.managed and not args.demo:
        from .session import BringupSession
        session = BringupSession(dict(mode='mock' if args.mock else 'real',
            allow_execution=args.allow_execution, real_config=args.real_config, cell=args.cell))
    window = Panel(cfg, backend, session, args.robot_ip, args.serial_port)
    window.show()
    try:
        app.exec_()
    finally:
        if session:
            try:
                session.stop()
            except Exception as exc:
                print('后端退出需要现场检查：'+str(exc), file=sys.stderr)
        if backend:
            backend.close()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
