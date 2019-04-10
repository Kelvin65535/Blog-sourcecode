---
title: PyTorch利用nn.Sequential优化网络搭建过程
date: 2019-04-10 14:43:28
tags:
 - PyTorch
---

本文以MobileNet为例，学习利用循环搭建大规模、重复程度大的深度网络结构的方法，像ResNet、Inception等大规模网络也可以利用本方法很方便构建一个很深的网络，值得学习。

<!-- more -->

以MobileNet为例，一开始的模型结构如下所示：

```python
class MobileNet(nn.Module):
    def __init__(self, num_classes=1000):
        super(MobileNet, self).__init__()
        self.Conv2d_s2 = BasicConv2d(3, 32, kernel_size=3, stride=2)
        self.Conv2d_dw_1 = DepthwiseConv(32, 64, kernel_size=3, stride=1)
        self.Conv2d_dw_2 = DepthwiseConv(64, 128, kernel_size=3, stride=2)
        self.Conv2d_dw_3 = DepthwiseConv(128, 128, kernel_size=3, stride=1)
        self.Conv2d_dw_4 = DepthwiseConv(128, 256, kernel_size=3, stride=2)
        self.Conv2d_dw_5 = DepthwiseConv(256, 256, kernel_size=3, stride=1)
        self.Conv2d_dw_6 = DepthwiseConv(256, 512, kernel_size=3, stride=2)
        # 7-11
        self.Conv2d_dw_7_11 = nn.Sequential(
            DepthwiseConv(512, 512, kernel_size=3, stride=1),
            DepthwiseConv(512, 512, kernel_size=3, stride=1),
            DepthwiseConv(512, 512, kernel_size=3, stride=1),
            DepthwiseConv(512, 512, kernel_size=3, stride=1),
            DepthwiseConv(512, 512, kernel_size=3, stride=1),
        )
        self.Conv2d_dw_12 = DepthwiseConv(512, 1024, kernel_size=3, stride=2)
        self.Conv2d_dw_13 = DepthwiseConv(1024, 1024, kernel_size=3, stride=2)
        # self.avgpool = F.avg_pool2d()
        self.fc = nn.Sequential(
            nn.Linear(1024, num_classes),
        )

    def forward(self, x):
        # input: 224 × 224 × 3
        x = self.Conv2d_s2(x)
        # input: 112 × 112 × 32
        x = self.Conv2d_dw_1(x)
        # input: 112 × 112 × 64
        x = self.Conv2d_dw_2(x)
        # input: 56 × 56 × 128
        x = self.Conv2d_dw_3(x)
        # input: 56 × 56 × 128
        x = self.Conv2d_dw_4(x)
        # input: 28 × 28 × 256
        x = self.Conv2d_dw_5(x)
        # input: 14 × 14 × 256
        x = self.Conv2d_dw_6(x)
        # input: 14 × 14 × 512
        x = self.Conv2d_dw_7_11(x)
        # input: 7 × 7 × 512
        x = self.Conv2d_dw_12(x)
        # input: 7 × 7 × 1024
        x = self.Conv2d_dw_13(x)
        # input: 7 × 7 × 1024
        x = F.avg_pool2d(x, 2)
        # input: 1 × 1 × 1024
        x = x.view(x.size(0), -1)
        # input: 1 × 1 × 1024
        x = self.fc(x)
        return x
```

对于比较深的网络，每一层的调用Conv2d方法显得很麻烦。PyTorch提供了`torch.nn.Sequential`容器用于将有序的模块打包成集合，添加到网络结构中。使用方法如下：

```python
nn.Sequential(
  nn.Conv2d(...),
  nn.ReLU(),
  ...,
  ...
)
```

同时使用`collection.OrderedDict`容器，支持分别带有命名的有序模块列表的传入：

```python
nn.Sequential(
  OrderedDict([
    ('conv1', nn.Conv2d(...)),
    ('relu1', nn.ReLU()),
    ('conv2', nn.Conv2d(...)),
    ('relu2', nn.ReLU())
  ])
)
```

由于Python对列表支持`*[List]`返回列表内的元素这种操作，因此可以使用循环的方式重复生成一系列的模块序列，如下所示：

```python
config = [...]  // 每一层的参数列表
// 循环读取config的参数，生成每一层
layers = []
for idx, param in enumerate(config):
  layers += nn.Conv2d(param)
  // 或者传入带命名的层
  layers += ('name', nn.Conv2d(param))
// 统一将layers里的模块传入nn.Sequential
model = nn.Sequential(*layers) // 当layers里的元素没有名字
model = nn.Sequential(OrderedDict(layers)) // 有名字
```

按照以上模式，重构MobileNet的`__init__()`方法：

```python
class MobileNet(nn.Module):
    def __init__(self, num_classes=1000):
        super(MobileNet, self).__init__()
        # (128, 2) = output channels 128, stride 2;
        # default stride is 1
        self.channel_cfg = [64, (128, 2), 128, (256,2), 256, (512,2), 512, 512, 512, 512, 512, (1024, 2), 1024]

        self.Conv2d_s2 = BasicConv2d(3, 32, kernel_size=3, stride=2)

        dw_layers = []
        in_channel = 32
        for idx, cfg in enumerate(self.channel_cfg):
            out_channel = cfg if isinstance(cfg, int) else cfg[0]
            stride = 1 if isinstance(cfg, int) else cfg[1]
            dw_layers.append(('conv_dw_'+str(idx), DepthwiseConv(in_channel, out_channel, kernel_size=3, stride=stride)))
            in_channel = out_channel

        self.Conv2d_dw = nn.Sequential(OrderedDict(dw_layers))

        self.fc = nn.Sequential(
            nn.Linear(1024, num_classes),
        )

    def forward(self, x):
        # input: 224 × 224 × 3
        x = self.Conv2d_s2(x)
        # # input: 112 × 112 × 32
        x = self.Conv2d_dw(x)
        # input: 7 × 7 × 1024
        x = F.avg_pool2d(x, 2)
        # input: 1 × 1 × 1024
        x = x.view(x.size(0), -1)
        # input: 1 × 1 × 1024
        x = self.fc(x)
        return x
```

关于怎么为模块列表添加名字，参考：

https://discuss.pytorch.org/t/how-can-i-give-a-name-to-each-module-in-modulelist/10547/5