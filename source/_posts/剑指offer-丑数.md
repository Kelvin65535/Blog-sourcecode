---
title: 剑指offer-丑数
date: 2017-07-26 10:21:49
tags: 
 - C++
 - 剑指offer
 - 算法题
---

## 题目

把只包含因子2、3和5的数称作丑数（Ugly Number）。例如6、8都是丑数，但14不是，因为它包含因子7。 习惯上我们把1当做是第一个丑数。求按从小到大的顺序的第N个丑数。

<!-- more -->

## 思路

由丑数的定义得，丑数只包含1或者只包含2、3、5的因子，因此可以推出`丑数 = 丑数 * (2、3、5其中一个因子)`。
假设丑数的序列为`a1, a2, a3, ...., an`，由此可得
`a1 = 1`，
包含2的有序丑数序列：`a1 * 2, a2 * 2, a3 * 2, ....`
包含3的有序丑数序列：`a1 * 3, a2 * 3, a3 * 3, ....`
包含3的有序丑数序列：`a1 * 5, a2 * 5, a3 * 5, ....`
且必定有`(an * 2) < (an * 3) < (an * 5)`，`(an * i) < (a(n+1) * i)`
因此将其组合成一个序列，即为丑数的序列：
`a1, a1*2, a1*3, a1*5, a2*2, a2*3, a2*5, ......`
可以用`t2`, `t3`, `t5`分别保存包含2、3、5的有序丑数数列的下标，即可解决。

## 代码
```cpp
class Solution {
public:
    int GetUglyNumber_Solution(int index) {
    	if(index <= 0)
            return 0;
        vector<int> result(index);//保存丑数的序列
        result[0] = 1;//相当于a[0]=1
        int t2 = 0, t3 = 0, t5 = 0;//分别保存包含因子2、3、5的丑数子序列的下标
        int i;
        for(i = 1; i < index; i++){
            result[i] = min(result[t2] * 2, min(result[t3] * 3, result[t5] * 5)); //找出对应下标的子序列里最小的丑数
            //根据result[i]所属的子序列，自增该子序列的下标
            if(result[i] == result[t2] * 2) t2++;
            if(result[i] == result[t3] * 3) t3++;
            if(result[i] == result[t5] * 5) t5++;
        }
        return result[index - 1];//返回序列的最后一个
    }
};
```

