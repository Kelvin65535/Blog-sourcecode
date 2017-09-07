---
title: 关于Underscore.js的map函数返回对象的问题
date: 2017-09-07 15:29:13
tags:
 - JavaScript
---

`Underscore.js`作为一个广受欢迎的JavaScript工具库，其内置了一整套函数式编程的使用功能。其中`map()`函数可以遍历指定的集合，对集合内每个元素执行指定的变换函数。官方文档对`map()`的介绍如下：

> **map** `_.map(list, iteratee, [context])` 
>
> 通过变换函数（**iteratee**迭代器）把**list**中的每个值映射到一个新的数组中（注：产生一个新的数组）。如果存在原生的**map**方法，就用原生**map**方法来代替。如果**list**是个JavaScript对象，**iteratee**的参数是`(value, key, list)`。


虽然`map()`函数接收一个数组或者集合作为输入，但是它永远只返回一个**数组**。如果对一个对象使用`map()`，那么返回的新集合将为原对象所有的**值**经过变换处理后组成的**数组**，而不是一个经过变换后的对象。

<!-- more -->

从`map()`函数的源码可以发现它的内部逻辑如下：

```javascript
_.map = _.collect = function (obj, iteratee, context) {
    //....
    results = Array(length); // 定义结果数组，length为obj长度
    //....
    for (var index = 0; index < length; index++) {
        // 若obj为对象，则currentKey为对象的key
        // 若obj为数组，则currentKey为数组的index
        var currentKey = keys ? keys[index] : index;
        result[index] = itaratee(obj[currentkey], currentKey, obj); // 对元素进行变换
    }
    return result; // 返回结果数组
}
```

通常我们会下意识的认为`map()`接收了一个对象作为输入，那经过迭代变换后的返回结果也应该是一个对象，但事实却并非如此。为避免这个问题，可以使用`Underscore.js`内置的`object()`函数，将经过`map()`转换的数组重新变换为对象：

```javascript
var newObj = _.object(
    _.map(obj, function (value, key) {
        return [key, value];
    })
);
```

或者

```javascript
var newObj = _.object(
    _.key(obj), // key
    _.map(obj, function (value, key) {
        return value;
    }) // value
);
```

其中`object()`函数的文档如下：

>**object** `_.object(list, [values])`
>
>将数组转换为对象。传递任何一个单独`[key, value]`对的列表，或者一个键的列表和一个值的列表。 如果存在重复键，最后一个值将被返回。



参考：https://stackoverflow.com/questions/19990275/return-object-from-map