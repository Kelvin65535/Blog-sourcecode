---
title: PyTorch入门学习笔记：Dataset和模型初探
date: 2019-04-03 19:16:24
tags:
 - PyTorch
---

## 一、一些有用的参考资料

PyTorch官方文档：https://pytorch.org/docs/stable/index.html

torchvision类似于tensorflow/models，包含了在计算机视觉领域主流的模型、数据以及图像处理函数。

torchvision官方文档：https://pytorch.org/docs/stable/torchvision/index.html

<!-- more -->

## 二、cifar数据集获取，如何使用数据集

PyTorch数据集被封装成`torch.utils.data.Dataset`类型，并且torchvision内的cifar数据集实现正是Dataset类型很好的一个实现实例，因此可以通过cifar数据集学习如何正确使用Dataset类型。

cifar数据集官网：https://www.cs.toronto.edu/~kriz/cifar.html

torchvision datasets文档：https://pytorch.org/docs/stable/torchvision/datasets.html，包含了torchvision已经实现的所有datasets的信息。

torchvision内置的cifar数据集实现：位于`torchvision.datasets.cifar`中，代码位于 https://pytorch.org/docs/master/_modules/torchvision/datasets/cifar.html

需要使用的import：

```python
from torchvision.datasets import cifar
from torch.utils.data import DataLoader
from torchvision import transforms
```

### Dataset类

一个典型的`Dataset`类可以抽象成如下的数据结构：

```python
class MyDataset(torch.utils.data.Dataset):
  def __init__(self, ...):
    # 初始化dataset
    self.data = ... # x data
    self.target = ... # y label
  def __getitem__(self, index):
    # 返回第index个数据
  def __len__(self):
    # 返回数据集样本量
```

Dataset类最重要的两个方法为`__getitem__`和`__len__`方法，分别返回第i个数据和数据量有多少。使用`__getitem__`的原因主要是为了将Dataset接入迭代器的考虑。也就是说，`Dataset`类相当于一个数据容器，PyTorch在模型上调用数据，会通过`torch.utils.data.DataLoader`类从Dataset中抽取数据，并输入模型。

综上，torchvision内置的Cafar的数据结构就容易理解了：`__init__`根据传入的参数设置数据集的属性，并且自动下载数据集；`__getitem__`返回img、target两个对象，分别是图像和对应的label。在构建自己的数据集时，可以参照上述的实现，自行继承Dataset类。

### DataLoader类

DataLoader位于`torch.util.data.DataLoader`中，最主要的作用是为模型提供数据的输入队列。它读取`Dataset`的输入，将输入按照特定的配置封装成batch，在PyTorch运行中需要从DataLoader中获取数据的时候，返回一个DataLoaderIter迭代器对象，这个迭代器内置`__next__`方法，在python循环中被调用，类比TensorFlow的`make_one_shot_iterator()`返回的迭代器。

需要注意的是，DataLoader只从Dataset中接受__tensor, number, dict, list__类型的数据，但是Dataset默认的构造方法并不会对原数据集进行任何预处理，也就是说仅获取数据集的原格式并不能直接供DataLoader使用，尤其是图像的PIL Image格式。

如果将Cifar数据集不带任何预处理直接接入DataLoader中，如下代码所示：

```python
dataloader = DataLoader(dataset, batch_size=1, shuffle=True)

for i, data in enumerate(dataloader):
    if i == 2: 
        break
    print(i)
    img, label = data
    print("data: ", data)
    print("label: ", label)
```

在迭代DataLoader时会报以下错误：

```python
TypeError: batch must contain tensors, numbers, dicts or lists; found <class 'PIL.Image.Image'>
```

根据https://discuss.pytorch.org/t/image-file-reading-typeerror-batch-must-contain-tensors-numbers-dicts-or-lists-found-class-pil-image-image/9909 所述：

> The error states that the DataLoader receives a PIL image. This is because there are no transforms made (`transform=None`) on the image. The `__getitem__` method of MyDataset passes an unprocessed PIL image to the DataLoader, whereas it should receive a tensor.

因此，Dataset需要将`__getitem__`返回的数据先通过`transform`预处理之后，才能正确被DataLoader读取。

修改Dataset构造代码，传入transform：

```python
transform = transforms.Compose([
    transforms.ToTensor()
])
dataset = cifar.CIFAR10(dataset_folder, True, download=True, transform=transform)
```

`transforms.ToTensor()`方法将一个`PIL Image`或者`numpy.ndarray`转换成一个tensor。

内部`__getitem__`读取transform的操作：

```python
def __getitem__(self, index):
  ...
  img = Image.fromarray(img)
  ...
  if self.transform is not None:
    img = self.transform(img)
  if self.target_transform is not None:
    target = self.target_transform(target)
  return img, target
```

经过ToTensor()后，调用Dataset获取item的操作：

```python
transform = transforms.Compose([
    transforms.ToTensor()
])
dataset = cifar.CIFAR10(dataset_folder, True, download=True, transform=transform)
for i in range(1):
    img, label = dataset.__getitem__(i)
    print("img dtype: ", img.dtype, ", and img shape: ", img.shape)
```

输出：

```python
img dtype:  torch.float32 , and img shape:  torch.Size([3, 32, 32])
```

可见dataset拿到的数据已经正确transform，可以顺利被DataLoader获取。

### transforms

transforms官方文档：https://pytorch.org/docs/stable/torchvision/transforms.html

几乎所有的数据使用前都需要预处理，transforms位于`torchvision`内，提供了绝大多数受支持的预处理操作。transforms使用`Compose`打包需要用到的Transform对象的列表，然后返回一个变换序列对象，该对象执行时接受一个Transform支持的输入，执行序列中的变换后，返回最后一个变换函数的输出。类似于imgaug的iaa.Sequential。

### 小结

1. Dataset类为数据集容器
2. DataLoader提供数据输入模型的管道
3. transforms提供了数据预处理方式

## 三、AlexNet模型的使用

等待补充

## 四、开始训练

等待补充

## 五、将模型部署到GPU上

等待补充