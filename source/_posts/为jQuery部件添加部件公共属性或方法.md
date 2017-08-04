---
title: 为jQuery部件添加部件公共属性或方法
date: 2017-08-04 16:53:49
tags:
 - JavaScript
 - jQuery
---

## 问题的提出

在jQuery中我们可以轻易创建一个新的插件或者扩展部件库中已有的插件，使用以下的方法：

```javascript
$.widget('namespace.widgetName', {
  // 在此处定义部件属性/方法
});
```

其中`namespace`为部件指定一个命名空间，`widgetName`为部件名，后面的对象包含扩展该组件构造函数的属性和方法。在当前构造器函数被执行后，这个自定义组件的构造函数将包含自定义的属性/方法，将被应用到每一个通过该构造器新建的实例中。

但是有个问题，由于此处的定义仅仅作为部件构造函数的定义，因此每一个新创建的部件实例将会添加上构造函数上定义的属性和方法，因此每一个属性都是部件唯一的，那如果要添加所有该部件都可用的公共属性或方法呢？这就需要修改部件的原型对象了。

## 操作步骤

要修改部件的原型对象，首先要找到该部件的原型对象。原型对象位于`$.widget.namespace.widgetName.prototype`中，其中`namespace`为部件的命名空间，`widgetName`为部件名。在定义一个新的部件后，jQuery会将该部件构造函数的原型添加到上述位置，因此通过添加或修改部件原型上的属性或方法，我们可以修改部件所有实例的行为。

## 示例

假如我们创建了一个自定义的进度条部件，定义如下：

```javascript
$.widget("custom.progressbar", {
  //...
});
```

我们要为这个部件添加一个公共的统计部件个数的功能，这个功能在每个部件实例都可以调用。

在java语言中，一个很简单的实现方法是为`progressbar`这个类添加一个静态属性`count`，通过修改该属性在新实例被创建后改变`count`的值，同时使用静态方法`getCount()`获取`count`属性的值。但是JavaScript语言没有类的概念，因此我们可以通过修改组件的原型对象来实现这个功能。

```javascript
$.widget("custom.progressbar", {
  _create: function () {
    // 在原型上添加属性，该属性将允许所有部件实例读取且不因实例的创建而改变
    if (!$.custom.progressbar.prototype.count) {
      $.custom.progressbar.prototype.count = 0;
    }
    // create a new widget
    $.custom.progressbar.prototype.count++; // 修改原型上的属性
    // 在原型上扩展方法，该方法将在所有部件实例上可用
    $.custom.progressbar.prototype.getCount() {
      return $.custom.progressbar.prototype.count;
    }
  }
})
```

通过上述配置后，`count`属性和`getCount()`方法将成为`progressbar`组件的“静态属性“和”静态方法“，这样便实现了为jQuery部件添加公共变量的功能，并且该方法不会污染其他的jQuery部件。