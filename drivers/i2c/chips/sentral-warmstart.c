/*
 * COPYRIGHT (C) 2015 PNI SENSOR CORPORATION
 *
 * LICENSED UNDER THE APACHE LICENSE, VERSION 2.0 (THE "LICENSE");
 * YOU MAY NOT USE THIS FILE EXCEPT IN COMPLIANCE WITH THE LICENSE.
 * YOU MAY OBTAIN A COPY OF THE LICENSE AT
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * THIS SOFTWARE IS PROVIDED BY PNI SENSOR CORPORATION "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL PNI SENSOR CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/notifier.h>
#include <linux/suspend.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/of_irq.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>

#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/dcache.h>
#include <linux/namei.h>
#include <linux/err.h>


#include "sentral-core.h"

#define DEBUG_WARM_DATA



enum sentral_param_number_warm_start {
    SEN_PARAM_WARM_START_XSTATES1 =   1,
    SEN_PARAM_WARM_START_XSTATES2 =   2,

    /* Raw Sensors (Page 2/ offset 127) */
    SEN_PARAM_WARM_START_RAW_MODE = 127,
    SEN_PARAM_WARM_START_MAX =      128,
};

typedef struct
{
    float q[4];
    float qe[4];
    float T[9];
    float x[9];
    float gbias[3];
    unsigned char initfilter;
    unsigned char qeUpdate;
    float gUpdateRatios[4];
    unsigned char isStill;
    unsigned char eachStill[3];
    float stillCount;
    float wasStill;
    int wait;
    int gbiascnt;
    unsigned char updateFlags[12];
    unsigned char biasMatch;
    unsigned char gbias_mode_change;
    float z[12];
    unsigned char gbias_mode;
    unsigned char resetRef;
    unsigned char resetRefStatus;
    float GyroThresh[6];
    float noiseLvls[9];
    float stillnessMode;
    float movingStdNsamples;
    unsigned char startmCal;
    signed char kStillOn;
    float kStime;
    float sscale;
    unsigned char control;
    float aDm;
    unsigned char sensorsON[3];
} k1_struct;

typedef struct
{
    unsigned char mcalStatus[2];
    float mcalScore[3];
    float mcalSIHI[12];
    signed char transientCompensation;
    float mcal1[3];
    float mcal2[3];
    float mRcal1;
    float mRcal2;
} mcalOut_struct;


static int warm_start_enable = 1;

struct sentral_param_page_item warm_start_page_items[SEN_PARAM_WARM_START_MAX] = {
    [1]  = {
        .param_size = sizeof(float),
        .param_name = "k1.x[6]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(001.xstates1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
////                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [2]  = {
        .param_size = sizeof(float),
        .param_name = "k1.x[8]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(002.xstates2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),
    },
    [3]  = {
        .param_size = sizeof(float),
        .param_name = "k1.noiseLvls[0]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(003.gparams1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [4]  = {
        .param_size = sizeof(float),
        .param_name = "k1.noiseLvls[1]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(004.gparams2_3, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [5]  = {
        .param_size = sizeof(float),
        .param_name = "k1.noiseLvls[2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(005.gparams4_5, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [6]  = {
        .param_size = sizeof(float),
        .param_name = "mcalOut.mcalSIHI[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(006.gparams6_7, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [7]  = {
        .param_size = sizeof(float),
        .param_name = "mcalOut.mcalSIHI[10]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(007.nparams1_2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [8]  = {
        .param_size = sizeof(float),
        .param_name = "mcalOut.mcalSIHI[11]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(008.nparams3_4, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [9]  = {
        .param_size = sizeof(unsigned char),
        .param_name = "mcalOut.mcalStatus[0]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(009.nparams5_6, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [10]  = {
        .param_size = sizeof(unsigned char),
        .param_name = "w.bg_cal_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(010.nparams7_8, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
////////////////
    [11]  = {
        .param_size = sizeof(float),
        .param_name = "mcalOut.mcalScore[2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(011.xstates1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [12]  = {
        .param_size = sizeof(float),
        .param_name = "mcalOut.mcalSIHI[11]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(012.xstates2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [13]  = {
        .param_size = sizeof(float),
        .param_name = "mcalOut.mcalSIHI[11]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(013.gparams1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [14]  = {
        .param_size = sizeof(float),
        .param_name = "mcalOut.mcalSIHI[11]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(014.gparams2_3, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    {0,}
/*
    [15]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(015.gparams4_5, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [16]  = {
        .param_size = 8,
        .param_name = "xstates[1], 4 Bytes, k1.x[7] -> kSet.x_init[7]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(016.gparams6_7, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [17]  = {
        .param_size = 8,
        .param_name = "xstates[2], 4 Bytes, k1.x[9] -> kSet.x_init[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(017.nparams1_2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [18]  = {
        .param_size = 8,
        .param_name = "gparams[1], 1 Bytes, k1.gbias_mode -> k1.gbias_mode,ktemp.gbias_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(018.nparams3_4, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [19]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(019.nparams5_6, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [20]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(020.nparams7_8, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },


///////
    [21]  = {
        .param_size = 8,
        .param_name = "xstates[1], 4 Bytes, k1.x[7] -> kSet.x_init[7]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(021.xstates1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [22]  = {
        .param_size = 4,
        .param_name = "xstates[2], 4 Bytes, k1.x[9] -> kSet.x_init[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(022.xstates2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [23]  = {
        .param_size = 8,
        .param_name = "gparams[1], 1 Bytes, k1.gbias_mode -> k1.gbias_mode,ktemp.gbias_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(023.gparams1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [24]  = {
        .param_size = 4,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(024.gparams2_3, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [25]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(025.gparams4_5, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [26]  = {
        .param_size = 4,
        .param_name = "xstates[1], 4 Bytes, k1.x[7] -> kSet.x_init[7]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(026.gparams6_7, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [27]  = {
        .param_size = 1,
        .param_name = "xstates[2], 4 Bytes, k1.x[9] -> kSet.x_init[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(027.nparams1_2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [28]  = {
        .param_size = 1,
        .param_name = "gparams[1], 1 Bytes, k1.gbias_mode -> k1.gbias_mode,ktemp.gbias_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(028.nparams3_4, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [29]  = {
        .param_size = 4,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(029.nparams5_6, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [30]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(030.nparams7_8, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },

//////
    [31]  = {
        .param_size = 8,
        .param_name = "xstates[1], 4 Bytes, k1.x[7] -> kSet.x_init[7]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(031.xstates1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [32]  = {
        .param_size = 8,
        .param_name = "xstates[2], 4 Bytes, k1.x[9] -> kSet.x_init[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(032.xstates2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [33]  = {
        .param_size = 8,
        .param_name = "gparams[1], 1 Bytes, k1.gbias_mode -> k1.gbias_mode,ktemp.gbias_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(033.gparams1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [34]  = {
        .param_size = 4,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(034.gparams2_3, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [35]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(035.gparams4_5, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [36]  = {
        .param_size = 4,
        .param_name = "xstates[1], 4 Bytes, k1.x[7] -> kSet.x_init[7]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(036.gparams6_7, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [37]  = {
        .param_size = 8,
        .param_name = "xstates[2], 4 Bytes, k1.x[9] -> kSet.x_init[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(037.nparams1_2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [38]  = {
        .param_size = 8,
        .param_name = "gparams[1], 1 Bytes, k1.gbias_mode -> k1.gbias_mode,ktemp.gbias_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(038.nparams3_4, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [39]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(039.nparams5_6, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [40]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(040.nparams7_8, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },



/////
    [41]  = {
        .param_size = 4,
        .param_name = "xstates[1], 4 Bytes, k1.x[7] -> kSet.x_init[7]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(041.xstates1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [42]  = {
        .param_size = 8,
        .param_name = "xstates[2], 4 Bytes, k1.x[9] -> kSet.x_init[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(042.xstates2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [43]  = {
        .param_size = 4,
        .param_name = "gparams[1], 1 Bytes, k1.gbias_mode -> k1.gbias_mode,ktemp.gbias_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(043.gparams1, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [44]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(044.gparams2_3, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [45]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(045.gparams4_5, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [46]  = {
        .param_size = 8,
        .param_name = "xstates[1], 4 Bytes, k1.x[7] -> kSet.x_init[7]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(046.gparams6_7, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [47]  = {
        .param_size = 8,
        .param_name = "xstates[2], 4 Bytes, k1.x[9] -> kSet.x_init[9]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(047.nparams1_2, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [48]  = {
        .param_size = 4,
        .param_name = "gparams[1], 1 Bytes, k1.gbias_mode -> k1.gbias_mode,ktemp.gbias_mode",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(048.nparams3_4, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [49]  = {
        .param_size = 8,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(049.nparams5_6, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
    [50]  = {
        .param_size = 4,
        .param_name = "gparams[2-3], 4 Bytes, k1.GyroThresh[1:2] -> k1.GyroThresh[1:2]",
        .param_data       = {0,},
//        .param_dev_attr = __ATTR(050.nparams7_8, S_IRUGO | S_IWUGO
//                         , sentral_sysfs_warm_start_page_item_show
//                         , sentral_sysfs_warm_start_page_item_store),                                 
    },
*/
};

int sentral_parameter_page_handle_internal(struct sentral_device *sentral
    , struct sentral_param_page_item items[], int mode)
{
    bool is_write;
    int item_size;
    int i, j;
    int page_number;
    int param_number;
    int rc;
    
    switch(mode)
    {
        case SEN_PARAM_PAGE_WARM_START_READ:
            item_size = SEN_PARAM_WARM_START_MAX;
            page_number =  SPP_ALGO_WARM_START;
            is_write = false;
            break;
        case SEN_PARAM_PAGE_WARM_START_WRITE:
            item_size = SEN_PARAM_WARM_START_MAX;
            page_number =  SPP_ALGO_WARM_START;
            is_write = true;
            break;
//        case SEN_PARAM_PAGE_KNOBS_READ:
//            item_size = SEN_PARAM_KNOBS_MAX;
//            is_write = false;
//            break;
//        case SEN_PARAM_PAGE_KNOBS_WRITE:
//            item_size = SEN_PARAM_KNOBS_MAX;
//            is_write = true;
//            break;
            
#ifdef SENTRAL_CUSTOM_PARAMPAGE
        case SEN_PARAM_PAGE_CUSTOM_READ:
            item_size = SEN_PARAM_CUSTOM_MAX;
            is_write = false;
            break;
        case SEN_PARAM_PAGE_CUSTOM_WRITE:
            item_size = SEN_PARAM_CUSTOM_MAX;
            is_write = true;
            break;
#endif            
        default:
            rc = -EINVAL;
            goto exit_error_page;
            break;
    }

    //items = sentral->page_items[page_number];

    //if (items==NULL)
    //{
    //    rc = -EINVAL;
    //    goto exit_error_page;
    //}
    
    // select page
    rc = sentral_write_byte(sentral, SR_PARAM_PAGE, page_number);
    if (rc) {
        LOGE(&sentral->client->dev, "error (%d) selecting parameter page: %u\n",
            rc, page_number);
        goto exit_error_page;
    }
    
    for(i=0; i<item_size; i++)
    {
        param_number = i;
        
        if (items[param_number].param_size==0)
        {
            continue;
        }

        if (is_write == true)
        {
            // write values
            rc = sentral_write_block(sentral, SR_PARAM_LOAD, items[param_number].param_data, items[param_number].param_size);
            if (rc < 0) {
                LOGE(&sentral->client->dev, "error (%d) writing parameter data\n", rc);
                goto exit_error_param;
            }

            param_number |= 0x80;
        }
        // select param number
        rc = sentral_write_byte(sentral, SR_PARAM_REQ, param_number);
        if (rc) {
            LOGE(&sentral->client->dev,
                    "error (%d) selecting parameter number: %u\n", rc,
                    param_number);
            goto exit_error_param;
        }
        
        // wait for ack
        for (j = 0; j < PARAM_MAX_RETRY; j++) {
            usleep_range(8000, 10000);
            rc = sentral_read_byte(sentral, SR_PARAM_ACK);
            if (rc < 0) {
                LOGE(&sentral->client->dev, "error (%d) reading parameter ack\n",
                        rc);
                goto exit;
            }
        
            if (rc == param_number)
                goto acked;
        }

        LOGE(&sentral->client->dev, "parameter ack retries (%d) exhausted\n",
                PARAM_MAX_RETRY);
    
        rc = -EIO;
        goto exit;
    
acked:
        if(is_write==false)
        {
            // read values
            rc = sentral_read_block(sentral, SR_PARAM_SAVE, items[param_number].param_data, items[param_number].param_size);
            if (rc < 0) {
                LOGE(&sentral->client->dev, "error (%d) reading parameter data\n", rc);
                goto exit;
            }
        }
        
    }
    
    rc = 0;

exit:
    (void)sentral_write_byte(sentral, SR_PARAM_REQ, 0);
exit_error_param:
    (void)sentral_write_byte(sentral, SR_PARAM_PAGE, 0);
exit_error_page:
    return rc;

}

ssize_t sentral_sysfs_warm_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    //struct iio_dev *indio_dev = dev_get_drvdata(dev);
    //struct sentral_device *sentral = iio_priv(indio_dev);
    
	struct sentral_device *sentral = dev_get_drvdata(dev);
    int count = 0;
    int rc=0, i;
    int item_size = SEN_PARAM_WARM_START_MAX;

    struct sentral_param_page_item *items = warm_start_page_items;

    if(warm_start_enable==0)
    {
        LOGE(&sentral->client->dev, "Disabled Warm start.\n");
        return count;
    }

	mutex_lock(&sentral->lock_pio);
    rc = sentral_parameter_page_handle_internal(sentral, warm_start_page_items, SEN_PARAM_PAGE_WARM_START_READ);
	mutex_unlock(&sentral->lock_pio);
    if (rc!=0)
    {
        LOGE(&sentral->client->dev, "Read warm start page error\n");
        return rc;
    }

    for (i=0; i< item_size; i++)
    {
        if (items[i].param_size==0)
        {
            continue;
        }
        count += scnprintf(buf+count, PAGE_SIZE-count, SENTRAL_PAGE_ITEM_IO_FORMAT"\n"
            , i, items[i].param_size, 0// 0:write data, 1: test data.
            , items[i].param_data[0], items[i].param_data[1]
            , items[i].param_data[2], items[i].param_data[3]
            , items[i].param_data[4], items[i].param_data[5]
            , items[i].param_data[6], items[i].param_data[7]
            );
    }

    // Write data into sentral.
    count += scnprintf(buf+count, PAGE_SIZE-count, SENTRAL_PAGE_ITEM_IO_FORMAT"\n"
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_NUMBER
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_SIZE
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_TEST
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_0
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_1
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_2
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_3
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_4
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_5
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_6
        , SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_7
        );

    if (count >= PAGE_SIZE)
    {
        // ERROR.
        LOGE(&sentral->client->dev, "count=%d\n", count);
    }
    else
    {
        LOGI(&sentral->client->dev, "count=%d\n", count);
    }

    return count;
}

ssize_t sentral_sysfs_warm_data_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    //struct iio_dev *indio_dev = dev_get_drvdata(dev);
    //struct sentral_device *sentral = iio_priv(indio_dev);
    
	struct sentral_device *sentral = dev_get_drvdata(dev);

    int n;
    int item_size = SEN_PARAM_WARM_START_MAX;
    int mode = SEN_PARAM_PAGE_WARM_START_WRITE;
    int param_number;
    int param_size;
    int param_test;
    int rc, i;
    unsigned char param_data[PARAM_READ_SIZE_MAX];
    struct sentral_param_page_item *items = warm_start_page_items;

    if(warm_start_enable==0)
    {
        LOGE(&sentral->client->dev, "Disabled Warm start.\n");
        return 0;
        //return count;
    }

    n = sscanf(buf, SENTRAL_PAGE_ITEM_IO_FORMAT
                    , &param_number, &param_size, &param_test
                    , (unsigned int*)&param_data[0], (unsigned int*)&param_data[1]
                    , (unsigned int*)&param_data[2], (unsigned int*)&param_data[3]
                    , (unsigned int*)&param_data[4], (unsigned int*)&param_data[5]
                    , (unsigned int*)&param_data[6], (unsigned int*)&param_data[7]
                    );
#ifdef DEBUG_WARM_DATA                    
    LOGI(&sentral->client->dev, SENTRAL_PAGE_ITEM_IO_FORMAT " n=%d\n"
                    , param_number, param_size, param_test
                    , param_data[0], param_data[1]
                    , param_data[2], param_data[3]
                    , param_data[4], param_data[5]
                    , param_data[6], param_data[7]
                    , n
                    );
#endif                    
                    
    if (n!=11)
    {
        LOGE(&sentral->client->dev, "the number of data invalid. n=%d\n", n);
        return -EINVAL;
    }

    if (
          param_number  == SENTRAL_PAGE_UPDATE_FLAG_PARAM_NUMBER
        &&param_size    == SENTRAL_PAGE_UPDATE_FLAG_PARAM_SIZE
        &&param_test    == SENTRAL_PAGE_UPDATE_FLAG_PARAM_TEST
        &&param_data[0] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_0
        &&param_data[1] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_1
        &&param_data[2] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_2
        &&param_data[3] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_3
        &&param_data[4] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_4
        &&param_data[5] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_5
        &&param_data[6] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_6
        &&param_data[7] == SENTRAL_PAGE_UPDATE_FLAG_PARAM_DATA_7
    )
    {
        mutex_lock(&sentral->lock_pio);
        rc = sentral_parameter_page_handle_internal(sentral, warm_start_page_items, SEN_PARAM_PAGE_WARM_START_WRITE);
        mutex_unlock(&sentral->lock_pio);

        if(rc<0)
        {
            LOGE(&sentral->client->dev, "Write Warm start failed.\n");
            return rc;
        }
        sentral->warm_start_ready = 1;

        return count;
    }


    if (items[param_number].param_size != param_size)
    {
        LOGE(&sentral->client->dev, "Data size mismatch.items[%d].param_size=%d(!=%d)\n"
            , param_number, items[param_number].param_size, param_size);
        return -EINVAL;
    }

    if(param_test==0)
    {
        items[param_number].param_data[0] = param_data[0];
        items[param_number].param_data[1] = param_data[1];
        items[param_number].param_data[2] = param_data[2];
        items[param_number].param_data[3] = param_data[3];
        items[param_number].param_data[4] = param_data[4];
        items[param_number].param_data[5] = param_data[5];
        items[param_number].param_data[6] = param_data[6];
        items[param_number].param_data[7] = param_data[7];
    }

    return count;
}

//static DEVICE_ATTR(data_warm, S_IRUGO | S_IWUGO, sentral_sysfs_warm_data_show, sentral_sysfs_warm_data_store);
ssize_t sentral_sysfs_warm_data_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    //struct iio_dev *indio_dev = dev_get_drvdata(dev);
    //struct sentral_device *sentral = iio_priv(indio_dev);
	struct sentral_device *sentral = dev_get_drvdata(dev);
    
    int rc;
    int input;

    rc = sscanf(buf, "%d", &input);
    if (rc) {
        LOGE(dev, "error (%d) parsing value\n", rc);
        return -EINVAL;
    }

    warm_start_enable = !!input;
    LOGI(&sentral->client->dev, "warm_start_enable=%d\n", warm_start_enable);

    return count;
}
ssize_t sentral_sysfs_warm_data_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    //struct iio_dev *indio_dev = dev_get_drvdata(dev);
    //struct sentral_device *sentral = iio_priv(indio_dev);
    int count = 0;

    count += scnprintf(buf+count, PAGE_SIZE-count, "%d\n", warm_start_enable);

    return count;
}
//static DEVICE_ATTR(data_warm_enable, S_IRUGO | S_IWUGO, sentral_sysfs_warm_data_enable_show, sentral_sysfs_warm_data_enable_store);

void sentral_warmstart_restore(struct sentral_device *sentral)
{
    if(sentral->warm_start_ready!=0)
    {
        int rc=0;

        mutex_lock(&sentral->lock_pio);
        rc = sentral_parameter_page_handle_internal(sentral, warm_start_page_items, SEN_PARAM_PAGE_WARM_START_WRITE);
        mutex_unlock(&sentral->lock_pio);
        if(rc<0)
        {
            LOGE(&sentral->client->dev, "Write Warm start failed.\n");
        }
    }

}


