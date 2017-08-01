---
title: 剑指offer-二叉搜索树的后序遍历序列
date: 2017-07-26 10:24:25
tags:
 - C++
 - 剑指offer
 - 算法题
---

## 题目

输入一个整数数组，判断该数组是不是某二叉搜索树的后序遍历的结果。如果是则输出Yes,否则输出No。假设输入的数组的任意两个数字都互不相同。

<!-- more -->

## 思路

BST（二叉搜索树）的后序序列的合法序列是，对于一个序列S，最后一个元素是root（也就是根）。
如果去掉一个元素的序列为T，那么T满足：T可以分成两段，前一段（左子树）小于root，后一段（右子树）大于root，且两段（子树）都是合法的后序序列。

## 代码

```cpp
class Solution {
public:
	//输入要判断的序列和序列的左、右元素位置，判断当前序列是否合法
    bool judge(vector<int> &a, int left, int right){
        if(left >= right)
            return true;//如果序列只有一个元素即合法，跳出递归
            
        //从后往前找到第一个小于root的元素，该元素与root之间的序列为右子树（均大于root）
        int i = right;
        while(i > left && a[i - 1] > a[right])//root为最后一个元素
            i--;
        //循环结束后，a[i]指向右子树的第一个元素
        
        //判断左子树是否所有的元素满足小于root的条件
        for(int j = i - 1; j >= left; j--){
            if(a[j] > a[right])
                return false;
        }
        //递归判断左子树、右子树序列，root均为序列的最后一个元素
        return judge(a, left, i - 1) && judge(a, i, right - 1);
    }
    
    bool VerifySquenceOfBST(vector<int> sequence) {
		if(sequence.size() == 0)
            return false;
        return judge(sequence, 0, sequence.size() - 1);
    }
    
};
```