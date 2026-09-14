"""Camera selection and freshness, independent of ROS and Tk."""


class PreviewState:
    def __init__(self):
        self.last_key = None

    def select(self, camera, sample, now, max_age=2.0):
        """Return whether to repaint, a fresh image, and any placeholder text."""
        image = None
        if sample is None:
            key, message = (camera, 'missing'), f'{camera}：等待相机图像'
        elif not 0 <= now-sample[1] <= max_age:
            key, message = (camera, 'stale'), f'{camera}：相机图像已过期/连接中断'
        elif sample[0].encoding not in ('rgb8', 'bgr8'):
            encoding = sample[0].encoding
            key = (camera, 'encoding', encoding)
            message = f'{camera}：不支持的相机图像格式 {encoding}'
        else:
            image, _ = sample
            key, message = (camera, id(image), sample[1]), ''
        changed = key != self.last_key
        self.last_key = key
        return changed, image, message
