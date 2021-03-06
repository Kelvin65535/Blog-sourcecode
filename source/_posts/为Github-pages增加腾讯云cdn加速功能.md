---
title: 为Github pages增加腾讯云cdn加速功能
date: 2017-08-01 17:19:44
tags:
 - 随笔
---

## 前言

有感于GitHub在国内的访问速度，于是便有使用cdn加速博客访问的想法，在腾讯云上一次测试成功。

<!-- more -->

## 操作步骤

你需要一个已经通过备案的个人域名来接入腾讯云的cdn服务，该域名可同时用于访问你的GitHub pages。

在cdn控制台中配置加速服务，域名为你的博客域名，源站类型设为域名，源站设置为由GitHub提供的GitHub pages访问域名，这样可以免除获取GitHub IP地址的麻烦。

![CDN域名配置](1.png)

接下来，在加速服务配置中，由于GitHub pages为静态页面，因此可选择静态加速，设置合适的缓存刷新时间后点击提交。

![加速服务配置](2.png)

按照提示等待大约5分钟后（实际所有cdn节点缓存页面生效的时间所需总共约半个小时，但仍可以继续配置），进入下一个配置项。

在开启cdn服务后，还需要配置回源host。在腾讯云帮助文档中可查询回源host的详细定义，简单的说回源host就是cdn访问源站时的域名，这里直接定义为个人域名即可。

![源站配置](3.png)

设置完成后，还需要修改域名的CNAME值，从而使加速域名能正常访问。在域名的DNS配置中，将域名对应的CNAME设置为cdn加速的CNAME值。

由于使用了国内cdn加速，在国外访问博客域名会很慢，因此可在CNAME配置中为国内、国外区域分别配置为cdn和GitHub的CNAME，从而达到更好的访问效果。

![CNAME配置](4.png)

## 验证

在命令行ping加速域名，若域名的后缀为cdntip.com或tcdn.qq.com则cdn加速成功。

![ping测试](5.png)

使用 [站长之家的网站ping测试]("http://ping.chinaz.com/") 来对比cdn加速的效果，在使用cdn加速前：

![未使用cdn](6.png)

使用cdn加速后：

![使用cdn](7.png)

可以看见祖国山河一片绿，形势一片大好（雾）。