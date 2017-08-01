---
title: 剑指offer-反转链表
date: 2017-07-26 10:26:37
tags:
 - C++
 - 剑指offer
 - 算法题
---

## 题目

输入一个链表，反转链表后，输出链表的所有元素。


链表结点结构如下：

```cpp
struct ListNode {
	int val;
	struct ListNode *next;
	ListNode(int x) :
			val(x), next(NULL) {
	}
};
```

<!-- more -->

## 思路

本题的目的是将链表里每个节点的`next`指针从指向下一个节点转为指向上一个节点，然后返回原链表最后一个节点的指针即可。

举个栗子：
原链表：`head -> a -> b -> c -> NULL`
反转后：`NULL <- a <- b <- c <- head`

因此，思路如下：
1. 始终将`pHead`指针指向当前要修改`next`指针的节点，使用`pre`和`next`指针，分别指向待修改节点的上一个、下一个节点
2. 将当前节点的`next`指针指向`pre`
3. `pre`指针移动到下一个节点（即当前节点），指向当前节点的指针移动到下一个节点
4. 移动到下一个要修改的节点后，将`next`指针指向`pHead->next`，重复2 - 3步骤
5. 直到当前节点为`null`为止，此时`pre`指针指向修改后链表的第一个节点

## 代码
```cpp
class Solution {
public:
    ListNode* ReverseList(ListNode* pHead) {
        if(pHead == NULL)
            return NULL;
		ListNode* pre = NULL;//pre用于指向当前修改节点的上一个
        ListNode* next = NULL;//next用于指向当前修改节点的下一个
        
        while(pHead != NULL){//始终维护pHead指针指向当前要修改的节点
            next = pHead->next;//next保存当前节点的下一个
            pHead->next = pre;//将当前节点的next指针修改为上一个
            pre = pHead;//pre下移
            pHead = next;//pHead下移
        }
        //完成后pHead指向为null，实际的首节点为pre
        return pre;
    }
};
```