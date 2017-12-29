/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：Fire_Kinetis_MCG_CFG.h
 * 描述         ：配置锁相环的列表，仅仅用于查找数据，不参与程序编译
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：野火嵌入式开发工作室
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/
#if	0
野火kinetis开发板超频配置一览表
根据 datasheet 所说的：PRDIV分频后范围在 2 MHz 到 4 MHz 之间.
PRDIV取值范围为：12~24       但实际上，PRDIV 取 11 也能正常运行
另外，VDIV取值范围为：0~31

频率为：50 / ( prdiv + 1 ) * ( mcg_div.vdiv + 24 )

频率: 109.091		PRDIV: 10		VDIV: 0 //不推荐 PRDIV:10
频率: 113.636		PRDIV: 10		VDIV: 1
频率: 118.182		PRDIV: 10		VDIV: 2
频率: 122.727		PRDIV: 10		VDIV: 3
频率: 127.273		PRDIV: 10		VDIV: 4
频率: 131.818		PRDIV: 10		VDIV: 5
频率: 136.364		PRDIV: 10		VDIV: 6
频率: 140.909		PRDIV: 10		VDIV: 7
频率: 145.455		PRDIV: 10		VDIV: 8
频率: 150		PRDIV: 10		VDIV: 9
频率: 154.545		PRDIV: 10		VDIV: 10
频率: 159.091		PRDIV: 10		VDIV: 11
频率: 163.636		PRDIV: 10		VDIV: 12
频率: 168.182		PRDIV: 10		VDIV: 13
频率: 172.727		PRDIV: 10		VDIV: 14
频率: 177.273		PRDIV: 10		VDIV: 15
频率: 181.818		PRDIV: 10		VDIV: 16
频率: 186.364		PRDIV: 10		VDIV: 17
频率: 190.909		PRDIV: 10		VDIV: 18
频率: 195.455		PRDIV: 10		VDIV: 19
频率: 200		PRDIV: 10		VDIV: 20
频率: 204.545		PRDIV: 10		VDIV: 21
频率: 209.091		PRDIV: 10		VDIV: 22
频率: 213.636		PRDIV: 10		VDIV: 23
频率: 218.182		PRDIV: 10		VDIV: 24
频率: 222.727		PRDIV: 10		VDIV: 25
频率: 227.273		PRDIV: 10		VDIV: 26
频率: 231.818		PRDIV: 10		VDIV: 27
频率: 236.364		PRDIV: 10		VDIV: 28
频率: 240.909		PRDIV: 10		VDIV: 29
频率: 245.455		PRDIV: 10		VDIV: 30
频率: 250		PRDIV: 10		VDIV: 31

频率: 100		PRDIV: 11		VDIV: 0
频率: 104.167		PRDIV: 11		VDIV: 1
频率: 108.333		PRDIV: 11		VDIV: 2
频率: 112.5		PRDIV: 11		VDIV: 3
频率: 116.667		PRDIV: 11		VDIV: 4
频率: 120.833		PRDIV: 11		VDIV: 5
频率: 125		PRDIV: 11		VDIV: 6
频率: 129.167		PRDIV: 11		VDIV: 7
频率: 133.333		PRDIV: 11		VDIV: 8
频率: 137.5		PRDIV: 11		VDIV: 9
频率: 141.667		PRDIV: 11		VDIV: 10
频率: 145.833		PRDIV: 11		VDIV: 11
频率: 150		PRDIV: 11		VDIV: 12
频率: 154.167		PRDIV: 11		VDIV: 13
频率: 158.333		PRDIV: 11		VDIV: 14
频率: 162.5		PRDIV: 11		VDIV: 15
频率: 166.667		PRDIV: 11		VDIV: 16
频率: 170.833		PRDIV: 11		VDIV: 17
频率: 175		PRDIV: 11		VDIV: 18
频率: 179.167		PRDIV: 11		VDIV: 19
频率: 183.333		PRDIV: 11		VDIV: 20
频率: 187.5		PRDIV: 11		VDIV: 21
频率: 191.667		PRDIV: 11		VDIV: 22
频率: 195.833		PRDIV: 11		VDIV: 23
频率: 200		PRDIV: 11		VDIV: 24
频率: 204.167		PRDIV: 11		VDIV: 25
频率: 208.333		PRDIV: 11		VDIV: 26
频率: 212.5		PRDIV: 11		VDIV: 27
频率: 216.667		PRDIV: 11		VDIV: 28
频率: 220.833		PRDIV: 11		VDIV: 29
频率: 225		PRDIV: 11		VDIV: 30
频率: 229.167		PRDIV: 11		VDIV: 31

频率: 92.3077		PRDIV: 12		VDIV: 0
频率: 96.1538		PRDIV: 12		VDIV: 1
频率: 100		PRDIV: 12		VDIV: 2
频率: 103.846		PRDIV: 12		VDIV: 3
频率: 107.692		PRDIV: 12		VDIV: 4
频率: 111.538		PRDIV: 12		VDIV: 5
频率: 115.385		PRDIV: 12		VDIV: 6
频率: 119.231		PRDIV: 12		VDIV: 7
频率: 123.077		PRDIV: 12		VDIV: 8
频率: 126.923		PRDIV: 12		VDIV: 9
频率: 130.769		PRDIV: 12		VDIV: 10
频率: 134.615		PRDIV: 12		VDIV: 11
频率: 138.462		PRDIV: 12		VDIV: 12
频率: 142.308		PRDIV: 12		VDIV: 13
频率: 146.154		PRDIV: 12		VDIV: 14
频率: 150		PRDIV: 12		VDIV: 15
频率: 153.846		PRDIV: 12		VDIV: 16
频率: 157.692		PRDIV: 12		VDIV: 17
频率: 161.538		PRDIV: 12		VDIV: 18
频率: 165.385		PRDIV: 12		VDIV: 19
频率: 169.231		PRDIV: 12		VDIV: 20
频率: 173.077		PRDIV: 12		VDIV: 21
频率: 176.923		PRDIV: 12		VDIV: 22
频率: 180.769		PRDIV: 12		VDIV: 23
频率: 184.615		PRDIV: 12		VDIV: 24
频率: 188.462		PRDIV: 12		VDIV: 25
频率: 192.308		PRDIV: 12		VDIV: 26
频率: 196.154		PRDIV: 12		VDIV: 27
频率: 200		PRDIV: 12		VDIV: 28
频率: 203.846		PRDIV: 12		VDIV: 29
频率: 207.692		PRDIV: 12		VDIV: 30
频率: 211.538		PRDIV: 12		VDIV: 31

频率: 85.7143		PRDIV: 13		VDIV: 0
频率: 89.2857		PRDIV: 13		VDIV: 1
频率: 92.8571		PRDIV: 13		VDIV: 2
频率: 96.4286		PRDIV: 13		VDIV: 3
频率: 100		PRDIV: 13		VDIV: 4
频率: 103.571		PRDIV: 13		VDIV: 5
频率: 107.143		PRDIV: 13		VDIV: 6
频率: 110.714		PRDIV: 13		VDIV: 7
频率: 114.286		PRDIV: 13		VDIV: 8
频率: 117.857		PRDIV: 13		VDIV: 9
频率: 121.429		PRDIV: 13		VDIV: 10
频率: 125		PRDIV: 13		VDIV: 11
频率: 128.571		PRDIV: 13		VDIV: 12
频率: 132.143		PRDIV: 13		VDIV: 13
频率: 135.714		PRDIV: 13		VDIV: 14
频率: 139.286		PRDIV: 13		VDIV: 15
频率: 142.857		PRDIV: 13		VDIV: 16
频率: 146.429		PRDIV: 13		VDIV: 17
频率: 150		PRDIV: 13		VDIV: 18
频率: 153.571		PRDIV: 13		VDIV: 19
频率: 157.143		PRDIV: 13		VDIV: 20
频率: 160.714		PRDIV: 13		VDIV: 21
频率: 164.286		PRDIV: 13		VDIV: 22
频率: 167.857		PRDIV: 13		VDIV: 23
频率: 171.429		PRDIV: 13		VDIV: 24
频率: 175		PRDIV: 13		VDIV: 25
频率: 178.571		PRDIV: 13		VDIV: 26
频率: 182.143		PRDIV: 13		VDIV: 27
频率: 185.714		PRDIV: 13		VDIV: 28
频率: 189.286		PRDIV: 13		VDIV: 29
频率: 192.857		PRDIV: 13		VDIV: 30
频率: 196.429		PRDIV: 13		VDIV: 31

频率: 80		PRDIV: 14		VDIV: 0
频率: 83.3333		PRDIV: 14		VDIV: 1
频率: 86.6667		PRDIV: 14		VDIV: 2
频率: 90		PRDIV: 14		VDIV: 3
频率: 93.3333		PRDIV: 14		VDIV: 4
频率: 96.6667		PRDIV: 14		VDIV: 5
频率: 100		PRDIV: 14		VDIV: 6
频率: 103.333		PRDIV: 14		VDIV: 7
频率: 106.667		PRDIV: 14		VDIV: 8
频率: 110		PRDIV: 14		VDIV: 9
频率: 113.333		PRDIV: 14		VDIV: 10
频率: 116.667		PRDIV: 14		VDIV: 11
频率: 120		PRDIV: 14		VDIV: 12
频率: 123.333		PRDIV: 14		VDIV: 13
频率: 126.667		PRDIV: 14		VDIV: 14
频率: 130		PRDIV: 14		VDIV: 15
频率: 133.333		PRDIV: 14		VDIV: 16
频率: 136.667		PRDIV: 14		VDIV: 17
频率: 140		PRDIV: 14		VDIV: 18
频率: 143.333		PRDIV: 14		VDIV: 19
频率: 146.667		PRDIV: 14		VDIV: 20
频率: 150		PRDIV: 14		VDIV: 21
频率: 153.333		PRDIV: 14		VDIV: 22
频率: 156.667		PRDIV: 14		VDIV: 23
频率: 160		PRDIV: 14		VDIV: 24
频率: 163.333		PRDIV: 14		VDIV: 25
频率: 166.667		PRDIV: 14		VDIV: 26
频率: 170		PRDIV: 14		VDIV: 27
频率: 173.333		PRDIV: 14		VDIV: 28
频率: 176.667		PRDIV: 14		VDIV: 29
频率: 180		PRDIV: 14		VDIV: 30
频率: 183.333		PRDIV: 14		VDIV: 31

频率: 75		PRDIV: 15		VDIV: 0
频率: 78.125		PRDIV: 15		VDIV: 1
频率: 81.25		PRDIV: 15		VDIV: 2
频率: 84.375		PRDIV: 15		VDIV: 3
频率: 87.5		PRDIV: 15		VDIV: 4
频率: 90.625		PRDIV: 15		VDIV: 5
频率: 93.75		PRDIV: 15		VDIV: 6
频率: 96.875		PRDIV: 15		VDIV: 7
频率: 100		PRDIV: 15		VDIV: 8
频率: 103.125		PRDIV: 15		VDIV: 9
频率: 106.25		PRDIV: 15		VDIV: 10
频率: 109.375		PRDIV: 15		VDIV: 11
频率: 112.5		PRDIV: 15		VDIV: 12
频率: 115.625		PRDIV: 15		VDIV: 13
频率: 118.75		PRDIV: 15		VDIV: 14
频率: 121.875		PRDIV: 15		VDIV: 15
频率: 125		PRDIV: 15		VDIV: 16
频率: 128.125		PRDIV: 15		VDIV: 17
频率: 131.25		PRDIV: 15		VDIV: 18
频率: 134.375		PRDIV: 15		VDIV: 19
频率: 137.5		PRDIV: 15		VDIV: 20
频率: 140.625		PRDIV: 15		VDIV: 21
频率: 143.75		PRDIV: 15		VDIV: 22
频率: 146.875		PRDIV: 15		VDIV: 23
频率: 150		PRDIV: 15		VDIV: 24
频率: 153.125		PRDIV: 15		VDIV: 25
频率: 156.25		PRDIV: 15		VDIV: 26
频率: 159.375		PRDIV: 15		VDIV: 27
频率: 162.5		PRDIV: 15		VDIV: 28
频率: 165.625		PRDIV: 15		VDIV: 29
频率: 168.75		PRDIV: 15		VDIV: 30
频率: 171.875		PRDIV: 15		VDIV: 31

频率: 70.5882		PRDIV: 16		VDIV: 0
频率: 73.5294		PRDIV: 16		VDIV: 1
频率: 76.4706		PRDIV: 16		VDIV: 2
频率: 79.4118		PRDIV: 16		VDIV: 3
频率: 82.3529		PRDIV: 16		VDIV: 4
频率: 85.2941		PRDIV: 16		VDIV: 5
频率: 88.2353		PRDIV: 16		VDIV: 6
频率: 91.1765		PRDIV: 16		VDIV: 7
频率: 94.1176		PRDIV: 16		VDIV: 8
频率: 97.0588		PRDIV: 16		VDIV: 9
频率: 100		PRDIV: 16		VDIV: 10
频率: 102.941		PRDIV: 16		VDIV: 11
频率: 105.882		PRDIV: 16		VDIV: 12
频率: 108.824		PRDIV: 16		VDIV: 13
频率: 111.765		PRDIV: 16		VDIV: 14
频率: 114.706		PRDIV: 16		VDIV: 15
频率: 117.647		PRDIV: 16		VDIV: 16
频率: 120.588		PRDIV: 16		VDIV: 17
频率: 123.529		PRDIV: 16		VDIV: 18
频率: 126.471		PRDIV: 16		VDIV: 19
频率: 129.412		PRDIV: 16		VDIV: 20
频率: 132.353		PRDIV: 16		VDIV: 21
频率: 135.294		PRDIV: 16		VDIV: 22
频率: 138.235		PRDIV: 16		VDIV: 23
频率: 141.176		PRDIV: 16		VDIV: 24
频率: 144.118		PRDIV: 16		VDIV: 25
频率: 147.059		PRDIV: 16		VDIV: 26
频率: 150		PRDIV: 16		VDIV: 27
频率: 152.941		PRDIV: 16		VDIV: 28
频率: 155.882		PRDIV: 16		VDIV: 29
频率: 158.824		PRDIV: 16		VDIV: 30
频率: 161.765		PRDIV: 16		VDIV: 31

频率: 66.6667		PRDIV: 17		VDIV: 0
频率: 69.4444		PRDIV: 17		VDIV: 1
频率: 72.2222		PRDIV: 17		VDIV: 2
频率: 75		PRDIV: 17		VDIV: 3
频率: 77.7778		PRDIV: 17		VDIV: 4
频率: 80.5556		PRDIV: 17		VDIV: 5
频率: 83.3333		PRDIV: 17		VDIV: 6
频率: 86.1111		PRDIV: 17		VDIV: 7
频率: 88.8889		PRDIV: 17		VDIV: 8
频率: 91.6667		PRDIV: 17		VDIV: 9
频率: 94.4444		PRDIV: 17		VDIV: 10
频率: 97.2222		PRDIV: 17		VDIV: 11
频率: 100		PRDIV: 17		VDIV: 12
频率: 102.778		PRDIV: 17		VDIV: 13
频率: 105.556		PRDIV: 17		VDIV: 14
频率: 108.333		PRDIV: 17		VDIV: 15
频率: 111.111		PRDIV: 17		VDIV: 16
频率: 113.889		PRDIV: 17		VDIV: 17
频率: 116.667		PRDIV: 17		VDIV: 18
频率: 119.444		PRDIV: 17		VDIV: 19
频率: 122.222		PRDIV: 17		VDIV: 20
频率: 125		PRDIV: 17		VDIV: 21
频率: 127.778		PRDIV: 17		VDIV: 22
频率: 130.556		PRDIV: 17		VDIV: 23
频率: 133.333		PRDIV: 17		VDIV: 24
频率: 136.111		PRDIV: 17		VDIV: 25
频率: 138.889		PRDIV: 17		VDIV: 26
频率: 141.667		PRDIV: 17		VDIV: 27
频率: 144.444		PRDIV: 17		VDIV: 28
频率: 147.222		PRDIV: 17		VDIV: 29
频率: 150		PRDIV: 17		VDIV: 30
频率: 152.778		PRDIV: 17		VDIV: 31

频率: 63.1579		PRDIV: 18		VDIV: 0
频率: 65.7895		PRDIV: 18		VDIV: 1
频率: 68.4211		PRDIV: 18		VDIV: 2
频率: 71.0526		PRDIV: 18		VDIV: 3
频率: 73.6842		PRDIV: 18		VDIV: 4
频率: 76.3158		PRDIV: 18		VDIV: 5
频率: 78.9474		PRDIV: 18		VDIV: 6
频率: 81.5789		PRDIV: 18		VDIV: 7
频率: 84.2105		PRDIV: 18		VDIV: 8
频率: 86.8421		PRDIV: 18		VDIV: 9
频率: 89.4737		PRDIV: 18		VDIV: 10
频率: 92.1053		PRDIV: 18		VDIV: 11
频率: 94.7368		PRDIV: 18		VDIV: 12
频率: 97.3684		PRDIV: 18		VDIV: 13
频率: 100		PRDIV: 18		VDIV: 14
频率: 102.632		PRDIV: 18		VDIV: 15
频率: 105.263		PRDIV: 18		VDIV: 16
频率: 107.895		PRDIV: 18		VDIV: 17
频率: 110.526		PRDIV: 18		VDIV: 18
频率: 113.158		PRDIV: 18		VDIV: 19
频率: 115.789		PRDIV: 18		VDIV: 20
频率: 118.421		PRDIV: 18		VDIV: 21
频率: 121.053		PRDIV: 18		VDIV: 22
频率: 123.684		PRDIV: 18		VDIV: 23
频率: 126.316		PRDIV: 18		VDIV: 24
频率: 128.947		PRDIV: 18		VDIV: 25
频率: 131.579		PRDIV: 18		VDIV: 26
频率: 134.211		PRDIV: 18		VDIV: 27
频率: 136.842		PRDIV: 18		VDIV: 28
频率: 139.474		PRDIV: 18		VDIV: 29
频率: 142.105		PRDIV: 18		VDIV: 30
频率: 144.737		PRDIV: 18		VDIV: 31

频率: 60		PRDIV: 19		VDIV: 0
频率: 62.5		PRDIV: 19		VDIV: 1
频率: 65		PRDIV: 19		VDIV: 2
频率: 67.5		PRDIV: 19		VDIV: 3
频率: 70		PRDIV: 19		VDIV: 4
频率: 72.5		PRDIV: 19		VDIV: 5
频率: 75		PRDIV: 19		VDIV: 6
频率: 77.5		PRDIV: 19		VDIV: 7
频率: 80		PRDIV: 19		VDIV: 8
频率: 82.5		PRDIV: 19		VDIV: 9
频率: 85		PRDIV: 19		VDIV: 10
频率: 87.5		PRDIV: 19		VDIV: 11
频率: 90		PRDIV: 19		VDIV: 12
频率: 92.5		PRDIV: 19		VDIV: 13
频率: 95		PRDIV: 19		VDIV: 14
频率: 97.5		PRDIV: 19		VDIV: 15
频率: 100		PRDIV: 19		VDIV: 16
频率: 102.5		PRDIV: 19		VDIV: 17
频率: 105		PRDIV: 19		VDIV: 18
频率: 107.5		PRDIV: 19		VDIV: 19
频率: 110		PRDIV: 19		VDIV: 20
频率: 112.5		PRDIV: 19		VDIV: 21
频率: 115		PRDIV: 19		VDIV: 22
频率: 117.5		PRDIV: 19		VDIV: 23
频率: 120		PRDIV: 19		VDIV: 24
频率: 122.5		PRDIV: 19		VDIV: 25
频率: 125		PRDIV: 19		VDIV: 26
频率: 127.5		PRDIV: 19		VDIV: 27
频率: 130		PRDIV: 19		VDIV: 28
频率: 132.5		PRDIV: 19		VDIV: 29
频率: 135		PRDIV: 19		VDIV: 30
频率: 137.5		PRDIV: 19		VDIV: 31

频率: 57.1429		PRDIV: 20		VDIV: 0
频率: 59.5238		PRDIV: 20		VDIV: 1
频率: 61.9048		PRDIV: 20		VDIV: 2
频率: 64.2857		PRDIV: 20		VDIV: 3
频率: 66.6667		PRDIV: 20		VDIV: 4
频率: 69.0476		PRDIV: 20		VDIV: 5
频率: 71.4286		PRDIV: 20		VDIV: 6
频率: 73.8095		PRDIV: 20		VDIV: 7
频率: 76.1905		PRDIV: 20		VDIV: 8
频率: 78.5714		PRDIV: 20		VDIV: 9
频率: 80.9524		PRDIV: 20		VDIV: 10
频率: 83.3333		PRDIV: 20		VDIV: 11
频率: 85.7143		PRDIV: 20		VDIV: 12
频率: 88.0952		PRDIV: 20		VDIV: 13
频率: 90.4762		PRDIV: 20		VDIV: 14
频率: 92.8571		PRDIV: 20		VDIV: 15
频率: 95.2381		PRDIV: 20		VDIV: 16
频率: 97.619		PRDIV: 20		VDIV: 17
频率: 100		PRDIV: 20		VDIV: 18
频率: 102.381		PRDIV: 20		VDIV: 19
频率: 104.762		PRDIV: 20		VDIV: 20
频率: 107.143		PRDIV: 20		VDIV: 21
频率: 109.524		PRDIV: 20		VDIV: 22
频率: 111.905		PRDIV: 20		VDIV: 23
频率: 114.286		PRDIV: 20		VDIV: 24
频率: 116.667		PRDIV: 20		VDIV: 25
频率: 119.048		PRDIV: 20		VDIV: 26
频率: 121.429		PRDIV: 20		VDIV: 27
频率: 123.81		PRDIV: 20		VDIV: 28
频率: 126.19		PRDIV: 20		VDIV: 29
频率: 128.571		PRDIV: 20		VDIV: 30
频率: 130.952		PRDIV: 20		VDIV: 31

频率: 54.5455		PRDIV: 21		VDIV: 0
频率: 56.8182		PRDIV: 21		VDIV: 1
频率: 59.0909		PRDIV: 21		VDIV: 2
频率: 61.3636		PRDIV: 21		VDIV: 3
频率: 63.6364		PRDIV: 21		VDIV: 4
频率: 65.9091		PRDIV: 21		VDIV: 5
频率: 68.1818		PRDIV: 21		VDIV: 6
频率: 70.4545		PRDIV: 21		VDIV: 7
频率: 72.7273		PRDIV: 21		VDIV: 8
频率: 75		PRDIV: 21		VDIV: 9
频率: 77.2727		PRDIV: 21		VDIV: 10
频率: 79.5455		PRDIV: 21		VDIV: 11
频率: 81.8182		PRDIV: 21		VDIV: 12
频率: 84.0909		PRDIV: 21		VDIV: 13
频率: 86.3636		PRDIV: 21		VDIV: 14
频率: 88.6364		PRDIV: 21		VDIV: 15
频率: 90.9091		PRDIV: 21		VDIV: 16
频率: 93.1818		PRDIV: 21		VDIV: 17
频率: 95.4545		PRDIV: 21		VDIV: 18
频率: 97.7273		PRDIV: 21		VDIV: 19
频率: 100		PRDIV: 21		VDIV: 20
频率: 102.273		PRDIV: 21		VDIV: 21
频率: 104.545		PRDIV: 21		VDIV: 22
频率: 106.818		PRDIV: 21		VDIV: 23
频率: 109.091		PRDIV: 21		VDIV: 24
频率: 111.364		PRDIV: 21		VDIV: 25
频率: 113.636		PRDIV: 21		VDIV: 26
频率: 115.909		PRDIV: 21		VDIV: 27
频率: 118.182		PRDIV: 21		VDIV: 28
频率: 120.455		PRDIV: 21		VDIV: 29
频率: 122.727		PRDIV: 21		VDIV: 30
频率: 125		PRDIV: 21		VDIV: 31

频率: 52.1739		PRDIV: 22		VDIV: 0
频率: 54.3478		PRDIV: 22		VDIV: 1
频率: 56.5217		PRDIV: 22		VDIV: 2
频率: 58.6957		PRDIV: 22		VDIV: 3
频率: 60.8696		PRDIV: 22		VDIV: 4
频率: 63.0435		PRDIV: 22		VDIV: 5
频率: 65.2174		PRDIV: 22		VDIV: 6
频率: 67.3913		PRDIV: 22		VDIV: 7
频率: 69.5652		PRDIV: 22		VDIV: 8
频率: 71.7391		PRDIV: 22		VDIV: 9
频率: 73.913		PRDIV: 22		VDIV: 10
频率: 76.087		PRDIV: 22		VDIV: 11
频率: 78.2609		PRDIV: 22		VDIV: 12
频率: 80.4348		PRDIV: 22		VDIV: 13
频率: 82.6087		PRDIV: 22		VDIV: 14
频率: 84.7826		PRDIV: 22		VDIV: 15
频率: 86.9565		PRDIV: 22		VDIV: 16
频率: 89.1304		PRDIV: 22		VDIV: 17
频率: 91.3043		PRDIV: 22		VDIV: 18
频率: 93.4783		PRDIV: 22		VDIV: 19
频率: 95.6522		PRDIV: 22		VDIV: 20
频率: 97.8261		PRDIV: 22		VDIV: 21
频率: 100		PRDIV: 22		VDIV: 22
频率: 102.174		PRDIV: 22		VDIV: 23
频率: 104.348		PRDIV: 22		VDIV: 24
频率: 106.522		PRDIV: 22		VDIV: 25
频率: 108.696		PRDIV: 22		VDIV: 26
频率: 110.87		PRDIV: 22		VDIV: 27
频率: 113.043		PRDIV: 22		VDIV: 28
频率: 115.217		PRDIV: 22		VDIV: 29
频率: 117.391		PRDIV: 22		VDIV: 30
频率: 119.565		PRDIV: 22		VDIV: 31

频率: 50		PRDIV: 23		VDIV: 0
频率: 52.0833		PRDIV: 23		VDIV: 1
频率: 54.1667		PRDIV: 23		VDIV: 2
频率: 56.25		PRDIV: 23		VDIV: 3
频率: 58.3333		PRDIV: 23		VDIV: 4
频率: 60.4167		PRDIV: 23		VDIV: 5
频率: 62.5		PRDIV: 23		VDIV: 6
频率: 64.5833		PRDIV: 23		VDIV: 7
频率: 66.6667		PRDIV: 23		VDIV: 8
频率: 68.75		PRDIV: 23		VDIV: 9
频率: 70.8333		PRDIV: 23		VDIV: 10
频率: 72.9167		PRDIV: 23		VDIV: 11
频率: 75		PRDIV: 23		VDIV: 12
频率: 77.0833		PRDIV: 23		VDIV: 13
频率: 79.1667		PRDIV: 23		VDIV: 14
频率: 81.25		PRDIV: 23		VDIV: 15
频率: 83.3333		PRDIV: 23		VDIV: 16
频率: 85.4167		PRDIV: 23		VDIV: 17
频率: 87.5		PRDIV: 23		VDIV: 18
频率: 89.5833		PRDIV: 23		VDIV: 19
频率: 91.6667		PRDIV: 23		VDIV: 20
频率: 93.75		PRDIV: 23		VDIV: 21
频率: 95.8333		PRDIV: 23		VDIV: 22
频率: 97.9167		PRDIV: 23		VDIV: 23
频率: 100		PRDIV: 23		VDIV: 24
频率: 102.083		PRDIV: 23		VDIV: 25
频率: 104.167		PRDIV: 23		VDIV: 26
频率: 106.25		PRDIV: 23		VDIV: 27
频率: 108.333		PRDIV: 23		VDIV: 28
频率: 110.417		PRDIV: 23		VDIV: 29
频率: 112.5		PRDIV: 23		VDIV: 30
频率: 114.583		PRDIV: 23		VDIV: 31

频率: 48		PRDIV: 24		VDIV: 0
频率: 50		PRDIV: 24		VDIV: 1
频率: 52		PRDIV: 24		VDIV: 2
频率: 54		PRDIV: 24		VDIV: 3
频率: 56		PRDIV: 24		VDIV: 4
频率: 58		PRDIV: 24		VDIV: 5
频率: 60		PRDIV: 24		VDIV: 6
频率: 62		PRDIV: 24		VDIV: 7
频率: 64		PRDIV: 24		VDIV: 8
频率: 66		PRDIV: 24		VDIV: 9
频率: 68		PRDIV: 24		VDIV: 10
频率: 70		PRDIV: 24		VDIV: 11
频率: 72		PRDIV: 24		VDIV: 12
频率: 74		PRDIV: 24		VDIV: 13
频率: 76		PRDIV: 24		VDIV: 14
频率: 78		PRDIV: 24		VDIV: 15
频率: 80		PRDIV: 24		VDIV: 16
频率: 82		PRDIV: 24		VDIV: 17
频率: 84		PRDIV: 24		VDIV: 18
频率: 86		PRDIV: 24		VDIV: 19
频率: 88		PRDIV: 24		VDIV: 20
频率: 90		PRDIV: 24		VDIV: 21
频率: 92		PRDIV: 24		VDIV: 22
频率: 94		PRDIV: 24		VDIV: 23
频率: 96		PRDIV: 24		VDIV: 24
频率: 98		PRDIV: 24		VDIV: 25
频率: 100		PRDIV: 24		VDIV: 26
频率: 102		PRDIV: 24		VDIV: 27
频率: 104		PRDIV: 24		VDIV: 28
频率: 106		PRDIV: 24		VDIV: 29
频率: 108		PRDIV: 24		VDIV: 30
频率: 110		PRDIV: 24		VDIV: 31
#endif


#ifndef _FIRE_KINETIS_MCG_CFG_
#define _FIRE_KINETIS_MCG_CFG_



#endif      //_FIRE_KINETIS_MCG_CFG_

