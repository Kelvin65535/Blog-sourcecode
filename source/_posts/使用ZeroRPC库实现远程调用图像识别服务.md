---
title: 使用ZeroRPC库实现远程调用图像识别服务
date: 2019-09-09 00:43:43
tags:
 - Python
---

[ZeroRPC](https://github.com/0rpc/zerorpc-python)是基于ZeroMQ的远程调用框架，它提供了方便的方式实现第三方调用本地服务的功能。在Baxter的视觉识别任务中，将训练完成的Mask RCNN模型在服务器中运行，连接机器人和ROS的客户端通过ZeroRPC调用服务器的识别服务，有效利用服务器的计算资源。由于在内网环境中，tcp连接和传输的时间可以满足摄像头捕捉的视频流的识别需求。

<!-- more -->

服务器的RPC服务端搭建：

```python
class MaskRCNN(object):
    def getMask(self, cvimage):
        start_time = time.time()
        # 反序列化传入的cvMat
        cvimage = string_to_cvmat(cvimage)
        # 调用Mask RCNN模型识别
        composite = coco_demo.run_on_opencv_image(cvimage)
        print("Time: {:.2f} s / img".format(time.time() - start_time))
        # 序列化后返回识别后的结果到客户端
        return cvmat_to_string(composite)
# 在主线程中启动ZeroRPC服务
s = zerorpc.Server(MaskRCNN())
s.bind("tcp://0.0.0.0:38282")
s.run()
```

机器人端的RPC客户端搭建：

```python
# 在主线程中启动ZeroRPC服务
c = zerorpc.Client()
c.connect("tcp://127.0.0.1:8282")
# 在ROS订阅的回调函数中更新摄像头传回的图像
self._image_sub = rospy.Subscriber("/cameras/right_hand_camera/image", Image, 	   self._image_callback)
self.mutex = threading.Lock()
# 回调函数，由于ROS的机制回调函数在新线程中执行，因此使用线程锁保护临界资源
def _image_callback(self, img_data):
  try:
    if self.mutex.acquire():
      self._original_image = self._bridge.imgmsg_to_cv2(img_data, "bgr8")
      self.mutex.release()
		except CvBridgeError as e:
			print e
# 在主线程中远程调用服务器计算，该方法只能在主线程中处理，不能在子线程中
def process(self):
		if self._original_image is not None:
			jpg_as_text = self.cvmat_to_string(self._original_image)
			result = self._rpc.getMask(jpg_as_text)
			composite = self.string_to_cvmat(result)
			cv2.imshow("COCO detections", composite)
			cv2.waitKey(1)
while True:
  process()
```

cvMat变量的序列化方式采用了转格式为jpg以字符串流的形式传输：

```python
def cvmat_to_string(cvmat):
    retval, buffer = cv2.imencode('.jpg', cvmat)
    jpg_as_text = base64.b64encode(buffer)
    return jpg_as_text

def string_to_cvmat(string):
    jpg_original = base64.b64decode(string)
    jpg_original = np.asarray(bytearray(jpg_original), dtype="uint8")
    cvimage = cv2.imdecode(jpg_original, cv2.IMREAD_COLOR)
    return cvimage
```

利用Baxter机器人的右手末端摄像头进行测试，在客户端的roscore中订阅摄像头发布的图像，然后将图像通过ZeroRPC发送到服务器中，识别后将结果返回客户端。运行结果如下所示：

![Baxter摄像头和图像识别服务的联动](Baxter摄像头和图像识别服务的联动.jpg)

![服务器计算图片后返回客户端](服务器计算图片后返回客户端.jpg)
