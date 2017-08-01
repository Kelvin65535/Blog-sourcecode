---
title: 剑指offer-求1+2+3+...+n
date: 2017-07-26 09:50:23
tags: 
 - 剑指offer
 - C++
 - 算法题
---

## 题目

求1+2+3+...+n，要求不能使用乘除法、for、while、if、else、switch、case等关键字及条件判断语句`（A?B:C）`。

<!-- more -->

## 思路
利用逻辑与（`&&`）运算的短路特性求解。

当逻辑表达式`a && b`中`a`取值为0时，整个表达式的取值为0，此时编译器将不去求表达式`b`的值。

因此，可以利用递归的方法，在表达式`b`中放入递归语句，利用`&&`的短路特性，当表达式`a`为0时，不执行表达式`b`中的递归语句，实现跳出递归的效果。

## 代码
```cpp
class Solution {
public:
    int Sum_Solution(int n) {
        int result = n;
        //利用&&的短路特性，当result为0时，不执行后面的递归语句，实现跳出递归
        result && (result += Sum_Solution(n - 1));
        return result;
    }
};
```