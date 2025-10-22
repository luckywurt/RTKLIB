- 转换GPS + GLONASS + Galileo + BDS（常见组合，MSM7）
```shell
rnx2rtcm my.obs -typ 1005,1033,1077,1087,1097,1127 -sta 100 -out my.rtcm3
```

- 转换RINEX导航文件为RTCM3
```shell
rnx2rtcm BRDM.rnx -typ 1019,1020,1042,1045,1046 -out BRDM.rtcm3 
```

- 将RTCM3转换为RINEX观测文件
```shell
convbin input.rtcm3 -o output.obs -od -os
```

```shell
./str2str -in /root/clion-remote/RTKLIB-rtklibexplore/bin/output/WUHN.rtcm3#rtcm3 -out tcpsvr://:2101
```