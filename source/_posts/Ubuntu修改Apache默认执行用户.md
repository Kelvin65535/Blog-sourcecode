---
title: Ubuntu修改Apache默认执行用户
date: 2017-09-01 09:44:32
tags:
 - Ubuntu
 - Apache
---

## 前言

Ubuntu的Apache默认执行用户是`www-data`，用户组是`www-data`，但是由于`www-data`为低权限用户，有时会出现使用php读取文件、执行脚本时出现权限不够的情况，因此可以通过修改Apache的配置，达到使用其他用户执行Apache的效果。

适用版本：Apache 2.4.7

<!-- more -->

## 配置说明

Apache的配置文件位于`/etc/apache2/`中，在`apache2.conf`里可以找到以下配置：

```shell
# These need to be set in /etc/apache2/envvars
User ${APACHE_RUN_USER}
Group ${APACHE_RUN_GROUP}
```

可见Apache的默认执行用户和用户组就存放于`envvars`文件的 `${APACHE_RUN_USER}`和`${APACHE_RUN_GROUP}`变量中，打开`envvers`文件可以找到以下配置：

```shell
# Since there is no sane way to get the parsed apache2 config in scripts, some
# settings are defined via environment variables and then used in apache2ctl,
# /etc/init.d/apache2, /etc/logrotate.d/apache2, etc.
export APACHE_RUN_USER=www-data
export APACHE_RUN_GROUP=www-data
```

以上就是Apache默认的用户和用户组，可以修改这两个变量来达到切换Apache执行用户的效果。

修改完成后，重启Apache服务即可生效。