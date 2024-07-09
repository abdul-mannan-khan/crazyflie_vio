# -*- coding: utf-8 -*-
#
# ,---------,       ____  _ __
# |  ,-^-,  |      / __ )(_) /_______________ _____  ___
# | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
# | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
# Copyright (C) 2023 Bitcraze AB
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, in version 3.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
Example of how to connect to a motion capture system and feed the position to a
Crazyflie, using the motioncapture library. The motioncapture library supports all major mocap systems and provides
a generalized API regardless of system type.
The script uses the high level commander to upload a trajectory to fly a figure 8.

Set the uri to the radio settings of the Crazyflie and modify the
mocap setting matching your system.
"""
import time
import sys
import os
from threading import Thread

import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# The host name or ip address of the mocap system
host_name = "192.168.0.50"
#host_name = ('0.0.0.0', 51001)

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'vicon'


# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'cf'

# True: send position and orientation; False: send position only
send_full_pose = True

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

data2 = """
1.07688	0	0	0	0	-1.15566	2.63383	-2.01901	0.529088	0	0	0	0	5.37394	-9.58098	6.30788	-1.48213	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.90153	0	0.169529	0.21318	0.0414901	1.94933	-2.59542	1.15678	-0.175042	0.7	1.05132	-0.0875039	-0.173284	-0.639621	0.602462	-0.187973	0.0197979	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.42916	1.29	0.00375868	-0.237577	-0.00508415	0.930695	-1.27698	0.682748	-0.130732	0.7	-0.856526	-0.0575224	-0.0435215	-3.1619	5.7287	-3.46365	0.707633	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.48024	1.29	0.150336	0.226495	0.00411554	5.74852	-9.40903	5.29593	-1.02013	-0.7	-0.409238	0.0856803	0.00149391	0.174168	0.0906595	-0.174864	0.049592	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.45427	2.58	0.261129	-0.210241	-0.00505752	-0.664742	1.14129	-0.638959	0.121706	-0.7	0.485713	0.130741	0.00933877	4.07755	-6.77436	3.87485	-0.758832	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.4767	2.58	0.0849756	0.190045	0.0139195	4.23436	-6.63175	3.63013	-0.68598	0.7	0.56781	-0.124562	0.0166976	-0.686255	0.603603	-0.169857	0.0108057	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.57466	3.87	0.754435	-0.125002	0.0333094	-0.777521	0.584466	-0.123723	-5.4623E-14	0.7	-0.551024	-0.114612	-0.0153388	-2.28837	3.56506	-1.89487	0.343815	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.45661	3.87	-0.754435	-0.125002	-0.0333094	-3.35693	6.03711	-3.59521	0.721743	-0.7	-0.551024	0.114612	-0.0153388	0.594025	-0.431337	0.0624887	0.0118071	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.45427	2.58	-0.135509	0.190045	-0.00626378	0.58491	-1.12162	0.667784	-0.132515	-0.7	0.56781	0.124562	0.0166976	3.94352	-6.66585	3.84998	-0.758832	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.4767	2.58	-0.222923	-0.210241	0.00734527	-5.57384	9.16213	-5.17643	1.0004	0.7	0.485713	-0.130741	0.00933877	-0.733611	0.828297	-0.343432	0.050359	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.44661	1.29	-0.230005	0.226495	-0.00213494	1.04706	-1.88239	1.10059	-0.217332	0.7	-0.409238	-0.0856803	0.00149391	-3.72869	5.96481	-3.35197	0.650733	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.56177	1.29	-0.234828	-0.237577	-0.0115796	-4.05443	6.38648	-3.42778	0.627568	-0.7	-0.856526	0.0575224	0.0435215	0.300246	0.28744	-0.391472	0.0995145	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.05477	0	-0.211911	0.21318	-0.0414901	1.85286	-4.29472	3.40131	-0.919235	-0.7	1.05132	0.0875039	-0.173284	1.74659	-6.25981	6.12368	-1.87605	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
"""


data = """
1.2769	0	0	0	0	0.788551	-1.15752	0.626392	-0.120539	0	0	0	0	4.64396	-8.42866	5.37495	-1.18363	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.722563	0.215	0.28153	0.000984603	0.0213249	1.11074	-3.833	4.49742	-1.79771	0.4808	0.198889	-0.0905154	-0.0312764	6.70994	-23.0217	27.2329	-10.9665	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.650433	0.43	0.292338	0.0201575	0.0015198	2.7733	-10.0544	12.7051	-5.52288	0.6081	0.0252749	0.0024724	0.0120966	4.32599	-16.1504	20.7516	-9.12379	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.641459	0.645	0.322319	0.00350735	0.00531894	2.74804	-10.7727	14.3692	-6.5105	0.645	0.0111957	-0.0272146	-0.00778034	-4.67607	16.8761	-21.3973	9.36955	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	0.86	0.303583	0.00832353	0.00257173	2.71241	-10.0409	12.432	-5.20059	0.6081	-0.0577952	-0.0155391	0.00335755	-10.3169	35.069	-41.5966	16.9541	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	1.075	0.240217	-0.0335835	0.00360192	0.502828	-1.2514	1.01454	-0.273642	0.4808	-0.113402	-0.0249083	-0.00230453	-6.74479	14.3947	-10.6917	2.72677	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	1.29	0.105717	0.000392372	0.00764546	0.854952	-1.65143	1.15144	-0.281097	0	-0.208791	0.000738697	0.00521613	-6.49789	14.0935	-10.5735	2.71481	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	1.505	0.238416	0.0333274	0.00274558	3.54661	-11.7954	13.7533	-5.53368	-0.4808	-0.116794	0.0244262	-0.000692544	-9.90649	34.4875	-41.4844	17.0661	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.643482	1.72	0.306524	0.00949536	0.00451196	2.80943	-10.4856	13.5519	-6.00485	-0.6081	-0.0522575	0.0177447	-0.000298237	-3.79194	14.64	-19.3393	8.69842	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.64263	1.935	0.316408	1.2986E-05	0.00119319	3.05021	-11.6398	15.2987	-6.86363	-0.645	6.62958E-05	0.0205813	-4.48388E-05	4.33981	-15.7352	20.0311	-8.79419	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	2.15	0.306566	0.0094522	0.00453104	2.73732	-10.3037	12.8887	-5.42955	-0.6081	0.0520645	0.0176388	0.00040038	10.4187	-35.3513	41.8744	-17.0508	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	2.365	0.238259	-0.019822	-0.001365	1.00611	-2.45735	1.97061	-0.527866	-0.4808	0.117314	0.0248646	0.000625694	6.71984	-14.3486	10.6616	-2.71988	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	2.58	0.0531963	-7.33569E-05	0.00375781	2.84711	-6.02794	4.45965	-1.13475	0	0.206723	4.37227E-06	-0.00481853	6.5131	-14.1239	10.5947	-2.71995	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	2.795	0.119466	0.0167842	-0.00152495	17.6435	-60.4538	72.008	-29.427	0.4808	0.117294	-0.0248674	0.000635228	9.89464	-34.4501	41.443	-17.0501	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.64263	3.01	0.152733	0.00450732	-0.00190066	23.5212	-87.786	113.792	-50.5782	0.6081	0.0520972	-0.0176258	0.000378631	3.82135	-14.77	19.5341	-8.79684	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.643482	3.225	0.15931	0.000670594	-0.00134341	23.3662	-87.2508	113.062	-50.2202	0.645	3.94758E-07	-0.0206208	-2.66992E-07	-4.31399	15.6197	-19.8565	8.70562	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	3.44	0.150581	-0.00675894	-0.000940565	17.2555	-59.4773	71.1114	-29.1353	0.6081	-0.0520984	-0.0176264	-0.000378023	-10.4181	35.3496	-41.8727	17.0502	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	3.655	0.12786	-0.008653	-0.0018122	2.94084	-6.48816	4.91272	-1.26873	0.4808	-0.117291	-0.0248648	-0.000635626	-6.71999	14.3488	-10.6618	2.71992	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	3.87	4.87137E-15	-0.0351568	3.75803E-15	-3.21964	6.7887	-5.00245	1.26873	0	-0.206736	-9.65066E-16	0.00482089	-6.51301	14.1237	-10.5946	2.71992	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	3.655	-0.12786	-0.008653	0.0018122	-17.4351	59.7849	-71.2583	29.1353	-0.4808	-0.117291	0.0248648	-0.000635626	-9.89471	34.4503	-41.4433	17.0502	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.643482	3.44	-0.150581	-0.00675894	0.000940565	-23.4546	87.4162	-113.148	50.2202	-0.6081	-0.0520984	0.0176264	-0.000378023	-3.79641	14.655	-19.3568	8.70562	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.64263	3.225	-0.15931	0.000670594	0.00134341	-23.4573	87.6677	-113.73	50.5782	-0.645	3.94758E-07	0.0206208	-2.66992E-07	4.34157	-15.7409	20.0377	-8.79684	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	3.01	-0.152733	0.00450732	0.00190066	-17.3746	59.9911	-71.7871	29.427	-0.6081	0.0520972	0.0176258	0.000378631	10.418	-35.3495	41.8725	-17.0501	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	2.795	-0.119466	0.0167842	0.00152495	-2.69022	5.85677	-4.40854	1.13475	-0.4808	0.117294	0.0248674	0.000635228	6.72004	-14.349	10.6619	-2.71995	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	2.58	-0.0531963	-7.33569E-05	-0.00375781	-2.84923	6.03342	-4.46409	1.13594	0	0.206723	-4.37227E-06	-0.00481853	6.51298	-14.1236	10.5944	-2.71988	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	2.365	-0.11913	-0.0167363	0.001365	-17.6419	60.4394	-71.9847	29.4159	0.4808	0.117314	-0.0248646	0.000625694	9.89474	-34.4509	41.4444	-17.0508	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.64263	2.15	-0.153283	-0.0047261	0.00226552	-23.53	87.8422	-113.881	50.6227	0.6081	0.0520645	-0.0176388	0.00040038	3.82082	-14.7667	19.5288	-8.79419	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.643482	1.935	-0.158204	-6.49301E-06	0.000596594	-23.336	87.0856	-112.811	50.0987	0.645	6.62958E-05	-0.0205813	-4.48388E-05	-4.31221	15.6099	-19.8417	8.69842	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	1.72	-0.153262	0.00474768	0.00225598	-17.3629	59.9558	-71.7498	29.4132	0.6081	-0.0522575	-0.0177447	-0.000298237	-10.4241	35.3767	-41.9091	17.0661	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	1.505	-0.119208	0.0166637	0.00137279	-2.69493	5.8674	-4.41653	1.13679	0.4808	-0.116794	-0.0244262	-0.000692544	-6.71161	14.326	-10.6429	2.71481	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.11644	1.29	-0.0528586	0.000196186	-0.00382273	-2.84607	6.02379	-4.45572	1.13362	0	-0.208791	-0.000738697	0.00521613	-6.51922	14.1487	-10.6182	2.72677	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.698071	1.075	-0.120109	-0.0167917	0.00180096	-17.6443	60.4716	-72.0401	29.4429	-0.4808	-0.113402	0.0249083	-0.00230453	-9.89026	34.3426	-41.2498	16.9541	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.641459	0.86	-0.151791	-0.00416177	0.00128587	-23.7173	88.6406	-115.08	51.2366	-0.6081	-0.0577952	0.0155391	0.00335755	-3.94075	15.4842	-20.674	9.36955	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.650433	0.645	-0.16116	-0.00175368	0.00265947	-22.2168	82.1566	-105.382	46.3244	-0.645	0.0111957	0.0272146	-0.00778034	4.38108	-16.2238	20.7893	-9.12379	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
0.694479	0.43	-0.146169	0.0100787	-0.000759902	-17.543	60.5969	-72.7116	29.918	-0.6081	0.0252749	-0.0024724	0.0120966	9.703	-31.933	37.2895	-15.0814	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
1.22288	0.215	-0.140765	-0.000492302	0.0106624	-1.85702	3.80879	-2.66726	0.634577	-0.4808	0.198889	0.0905154	-0.0312764	4.84676	-9.90167	6.90697	-1.6382	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0
"""

# my_trajectory = [list(map(float, line.split())) for line in data.strip().split('\n')]

my_trajectory = [
    [float(value) if 'E' not in value else 0 for value in line.split()]
    for line in data.strip().split('\n')
]

my_trajectory2 = [
    [float(value) if 'E' not in value else 0 for value in line.split()]
    for line in data2.strip().split('\n')
]
# print(my_trajectory)

class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print("I am here.")
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        print(pos)
                        time.sleep(5)
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def send_extpose_quat(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    # time.sleep(1)
    wait_for_position_estimator(cf)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
    #print("so far so good.")
    cflib.crtp.init_drivers()
    print("I successfully initialized drivers.")

    # Connect to the mocap system
    mocap_wrapper = MocapWrapper(rigid_body_name)
    print("so far so good.")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1

        while True:
            try:
                # Set up a callback to handle data from the mocap system
                mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])
                time.sleep(3)
                adjust_orientation_sensitivity(cf)
                activate_kalman_estimator(cf)
                # activate_mellinger_controller(cf)
                # duration = upload_trajectory(cf, trajectory_id, figure8)
                # duration = upload_trajectory(cf, trajectory_id, my_trajectory)
                duration = upload_trajectory(cf, trajectory_id, my_trajectory2)
                
                print('The sequence is {:.1f} seconds long'.format(duration))
                reset_estimator(cf)
                run_sequence(cf, trajectory_id, duration)
            except(KeyboardInterrupt,SystemExit):
                mocap_wrapper.close()
                print("Socket error!")
                sys.exit()
            

    mocap_wrapper.close()
