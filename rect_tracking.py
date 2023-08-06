import sensor, image, time, math, pyb
from pyb import Servo
from pid import PID

s1 = Servo(1) # P7
s2 = Servo(2) # P8

s1 = Servo(1)
s2 = Servo(2)

s1.angle(0)#舵机回正
s2.angle(0)

delay = 4000 #超时未识别到激光点,自动认为激光点位置在上一个矩形的角,防止激光已经打到黑胶布上了,却找不到激光


EXPOSURE_TIME_SCALE = 0.15 # 更改此乘数降低曝光  0.2差不多了,不要太低

pth = 1 # 比这个小的不识别 pixel_threshold
ath = 2

threshold_index = 0 # 0红, 1绿, 2蓝 #(30, 100, 10, 127, -40, 127)
thresholds = [(30, 100, 10, 127, -40, 127), # 红激光阈值,阈值助手中调  (30, 100, 10, 127, -40, 127)
              (50, 100, -90, 15, -74, 127), # 绿激光阈值
              (0, 30, 0, 64, -128, 0)] # 蓝卡片阈值

max_red_size = 0 # 红像素数大于这个值就认为是最大红点,以下同理
blob_red_elo = 0.0 # 红点方度

# PID参数部分
output_x = PID(p=0.3, i=1, d=0.9) #实例化PID对象   0.3  0.4
output_y = PID(p=0.4, i=1, d=0.9) #可调!!!!!!!!!!!!!!!!!


scale_x = 0.2 #TO DO           注意别忘啦
scale_y = 0.2


err_x = 1 #与目标点的距离
err_y = 1


sensor.reset() #初始化感光元件
sensor.set_pixformat(sensor.RGB565) # 识别外边框可以考虑暂时设成灰度GRAYSCALE,要使用激光时设成彩色RGB565
sensor.set_framesize(sensor.VGA) # QQVGA 160x120 WQXGA2 2592x1944 后续通过看图像大小影不影响帧率决定
sensor.skip_frames(time = 0) # 跳过刚开始不稳定的帧
clock = time.clock() #to do

# --------------------------------------------------------------------------
while(True):
# /--------------------------------------------------------------------------
    clock.tick() # 开始跟踪运行时间
    img = sensor.snapshot() # 拍摄一张照片，返回一个image对象, 在下方处理该帧

    # 下面的`threshold`应设置为足够高的值，以滤除在图像中检测到的具有
    # 低边缘幅度的噪声矩形。最适用与背景形成鲜明对比的矩形。
    # 越大矩形越少

    max_rect = None # [拍一张照,识别出来多个矩形但只找一个最大矩形,使用if语句检查当前矩形的面积是否大于max_rect的面积（如果max_rect不为None）。如果是none，则将max_rect更新为当前矩形
    for r in img.find_rects(threshold = 42000): # 遍历找到的所有矩形  41000
        if max_rect is None or r.magnitude() > max_rect.magnitude(): #则将最大的矩形更新为max_rect, 找不到矩形就把当前矩形赋上去
            max_rect = r # 暂时改成小于

# ---------------------------------------------------------------------------------
    if max_rect is not None: # 检查是否找到最大矩形,找到了就绘制出来
        img.draw_rectangle(max_rect.rect(), color = (0, 0, 255)) # 画边
        print("已找到矩形!")
        for p in max_rect.corners():
            img.draw_circle(p[0], p[1], 5, color = (0, 255, 0)) # 画点

            print(p) # 逆时针从左下角开始打印四个点坐标
        print("--------------------")
# /--------------------------------------------------------------------------------
    if max_rect is not None: # 拍照直到有矩形为止
        break # max_rect中已记录四个点的坐标,可通过max_rect.corners()[0][0]来访问左下角的x坐标


# 这里拿到了矩形的四个点坐标,从左下角开始逆时针0123
x0 = max_rect.corners()[0][0] # A4矩形从左下角逆时针开始数的四个角的坐标, 此处已经记下矩形位置.不应该再挪动云台
y0 = max_rect.corners()[0][1] # 时间不够就别乱用for循环简化了
x1 = max_rect.corners()[1][0]
y1 = max_rect.corners()[1][1]
x2 = max_rect.corners()[2][0]
y2 = max_rect.corners()[2][1]
x3 = max_rect.corners()[3][0]
y3 = max_rect.corners()[3][1]



# --------------------------------------------------------------------------------------------


sensor.set_auto_gain(False) # 关闭自动增益
sensor.set_auto_whitebal(False) # 关闭白平衡,后续都不需要识别边框了,关着好了

sensor.set_auto_exposure(False, exposure_us = int(sensor.get_exposure_us() * EXPOSURE_TIME_SCALE)) # 降低曝光


# 防止找到黑胶布上, 阈值放宽
thresholds[0] = threshold=(30, 100, 10, 127, -40, 127) # 可调!!!!!!!
#thresholds[1] = (3, 100, -7, 40, -40, 30)   (20, 100, -7, 40, -40, 30)
img.binary([thresholds[threshold_index]])
sensor.skip_frames(time = 300) # 跳过刚开始不稳定的帧

# -------等待至出现激光,并得到激光的坐标,后面要每隔多少时间获取一次激光的坐标,直到激光的坐标和点的坐标之差在 多少 以内停下, 走下一个角, 最后回到原点, 出现 蓝色信号 又执行一次


# -----------------------------------------第一次------------------------------------------------

print("第一次:")
print("等待出现红色激光...")
max_red_blob = None # 存放最大红块
while(True):
    img = sensor.snapshot() # 拍张照

    for blob in img.find_blobs([thresholds[0]], pixels_threshold=pth, area_threshold=ath, merge=False):
        if blob.elongation()> blob_red_elo and blob.pixels() > max_red_size: # 筛选要圆的,而且还得是最大的
            max_red_blob = blob
            max_red_size = blob.pixels() # 遍历比较

            img.draw_cross(max_red_blob.cx(), max_red_blob.cy())

    if max_red_blob is not None: # 拍照直到有红色激光为止
        break # 退出while循环

print("已找到红色激光!")

cx = max_red_blob.cx() #历史坐标
cy = max_red_blob.cy()





n = 0
#云台+1是左转  舵机+1是上转
time_start = pyb.millis()
while(True):

    err_x = x0 - cx   #两色点间x距离
    err_y = y0 - cy
    n = n + 1
    print("第几轮")
    print(n)
    print("起始点")
    print(cx)
    print(cy)
    print("目标点")
    print(x0)
    print(y0)
    print("当前误差")
    print(err_x)
    print(err_y)

    if (x0 - cx > 0): #在右边
        #当前角度 - 应该变化的距离, PID算法输出的误差值
        s1.angle(s1.angle() - abs(output_x.get_pid(err_x,scale_x))  ) #右转

    if (x0 - cx < 0): #在左边
        s1.angle(s1.angle() + abs(output_x.get_pid(err_x,scale_x))  )


#同理
    if (y0 - cy > 0): #在下面
        s2.angle(s2.angle() - abs(output_y.get_pid(err_y,scale_y))  )

    if (y0 - cy < 0): #在上面
        s2.angle(s2.angle() + abs(output_y.get_pid(err_y,scale_y))  )

    time.sleep_ms(200) # 这个也很重要别忘了!!!


    max_red_blob = None # 存放最大红块
    while(True):

        if (pyb.millis() - time_start > delay):  #运行超过3秒,自动赋上一个角的坐标给色块当前坐标,并退出
            print("运行时间")
            print(time_start)
            print(pyb.millis())
            cx = x0
            cy = y0
            print("超时")
            break

        img = sensor.snapshot() # 拍张照

        for blob in img.find_blobs([thresholds[threshold_index]],pixels_threshold=pth, area_threshold=ath, merge=False):
            if blob.elongation()> blob_red_elo and blob.pixels() > max_red_size: # 筛选要圆的,而且还得是最大的
                max_red_blob = blob
                max_red_size = blob.pixels() # 遍历比较

                img.draw_cross(max_red_blob.cx(), max_red_blob.cy())

        if max_red_blob is not None: # 拍照直到有红色激光为止
            cx = max_red_blob.cx()   #注释掉莫名其妙就能用了
            cy = max_red_blob.cy()
            break # 退出while循环                             这里记得写超时部分

    print("---------------------------")





    #output_x.reset_I()
    #output_y.reset_I()
    if(abs(x0 - cx)  > 1  and abs(y0 - cy)  > 1): # 反馈,再获取一次色块坐标,取绝对值
        continue # 继续靠近
    else:
        break

print("已经追到嘞!")

time.sleep_ms(1000)


## -------------------------------------------第二次---------------------------------------------

print("第二次:")

n = 0
#云台+1是左转  舵机+1是上转
time_start = pyb.millis()
while(True):

    err_x = x3 - cx   #两色点间x距离
    err_y = y3 - cy
    n = n + 1
    print("第几轮")
    print(n)
    print("起始点")
    print(cx)                      #试试写死
    print(cy)
    print("目标点")
    print(x3)
    print(y3)
    print("当前误差")
    print(err_x)
    print(err_y)

    if (x3 - cx > 0): #在右边
        #当前角度 - 应该变化的距离, PID算法输出的误差值
        s1.angle(s1.angle() - abs(output_x.get_pid(err_x,scale_x))  ) #右转

    if (x3 - cx < 0): #在左边
        s1.angle(s1.angle() + abs(output_x.get_pid(err_x,scale_x))  )


#同理
    if (y3 - cy > 0): #在下面
        s2.angle(s2.angle() - abs(output_y.get_pid(err_y,scale_y))  )

    if (y3 - cy < 0): #在上面
        s2.angle(s2.angle() + abs(output_y.get_pid(err_y,scale_y))  )

    time.sleep_ms(200) # 这个也很重要别忘了!!!


    max_red_blob = None # 存放最大红块
    while(True):

        if (pyb.millis() - time_start > delay):  #运行超过3秒,自动赋上一个角的坐标给色块当前坐标,并退出
            cx = x3
            cy = y3
            print("超时")
            break

        img = sensor.snapshot() # 拍张照

        for blob in img.find_blobs([thresholds[threshold_index]],pixels_threshold=pth, area_threshold=ath, merge=False):
            if blob.elongation()> blob_red_elo and blob.pixels() > max_red_size: # 筛选要圆的,而且还得是最大的
                max_red_blob = blob
                max_red_size = blob.pixels() # 遍历比较

                img.draw_cross(max_red_blob.cx(), max_red_blob.cy())

        if max_red_blob is not None: # 拍照直到有红色激光为止

            cx = max_red_blob.cx()   #注释掉莫名其妙就能用了
            cy = max_red_blob.cy()
            break # 退出while循环                             这里记得写超时部分

    print("---------------------------")



    #output_x.reset_I()
    #output_y.reset_I()
    if(abs(x3 - cx)  > 5  and abs(y3 - cy)  > 5): # 反馈,再获取一次色块坐标,取绝对值
        continue # 继续靠近
    else:
        break
print("已经追到嘞!")




time.sleep_ms(1000)




# -------------------------------------------第三次------------------------------------------------

print("第三次:")

n = 0
#云台+1是左转  舵机+1是上转
time_start = pyb.millis()
while(True):

    err_x = x2 - cx   #两色点间x距离
    err_y = y2 - cy
    n = n + 1
    print("第几轮")
    print(n)
    print("起始点")
    print(cx)
    print(cy)
    print("目标点")
    print(x2)
    print(y2)
    print("当前误差")
    print(err_x)
    print(err_y)

    if (x2 - cx > 0): #在右边
        #当前角度 - 应该变化的距离, PID算法输出的误差值
        s1.angle(s1.angle() - abs(output_x.get_pid(err_x,scale_x))  ) #右转

    if (x2 - cx < 0): #在左边
        s1.angle(s1.angle() + abs(output_x.get_pid(err_x,scale_x))  )


#同理
    if (y2 - cy > 0): #在下面
        s2.angle(s2.angle() - abs(output_y.get_pid(err_y,scale_y))  )

    if (y2 - cy < 0): #在上面
        s2.angle(s2.angle() + abs(output_y.get_pid(err_y,scale_y))  )

    time.sleep_ms(200) # 这个也很重要别忘了!!!


    max_red_blob = None # 存放最大红块
    while(True):

        if (pyb.millis() - time_start > delay):  #运行超过3秒,自动赋上一个角的坐标给色块当前坐标,并退出
            cx = x2
            cy = y2
            print("超时")
            break

        img = sensor.snapshot() # 拍张照

        for blob in img.find_blobs([thresholds[threshold_index]],pixels_threshold=pth, area_threshold=ath, merge=False):
            if blob.elongation()> blob_red_elo and blob.pixels() > max_red_size: # 筛选要圆的,而且还得是最大的
                max_red_blob = blob
                max_red_size = blob.pixels() # 遍历比较

                img.draw_cross(max_red_blob.cx(), max_red_blob.cy())

        if max_red_blob is not None: # 拍照直到有红色激光为止
            cx = max_red_blob.cx()   #注释掉莫名其妙就能用了
            cy = max_red_blob.cy()

            break # 退出while循环                             这里记得写超时部分

    print("---------------------------")




    #output_x.reset_I()
    #output_y.reset_I()
    if(abs(x2 - cx)  > 5  and abs(y2 - cy)  > 5): # 反馈,再获取一次色块坐标,取绝对值
        continue # 继续靠近
    else:
        break
print("已经追到嘞!")


time.sleep_ms(1000)

# -------------------------------------------第四次--------------------------------------------------
print("第二次:")

n = 0
#云台+1是左转  舵机+1是上转

while(True):

    err_x = x1 - cx   #两色点间x距离
    err_y = y1 - cy
    n = n + 1
    print("第几轮")
    print(n)
    print("起始点")
    print(cx)
    print(cy)
    print("目标点")
    print(x1)
    print(y1)
    print("当前误差")
    print(err_x)
    print(err_y)

    if (x1 - cx > 0): #在右边
        #当前角度 - 应该变化的距离, PID算法输出的误差值
        s1.angle(s1.angle() - abs(output_x.get_pid(err_x,scale_x))  ) #右转

    if (x1 - cx < 0): #在左边
        s1.angle(s1.angle() + abs(output_x.get_pid(err_x,scale_x))  )


#同理
    if (y1 - cy > 0): #在下面
        s2.angle(s2.angle() - abs(output_y.get_pid(err_y,scale_y))  )

    if (y1 - cy < 0): #在上面
        s2.angle(s2.angle() + abs(output_y.get_pid(err_y,scale_y))  )

    time.sleep_ms(200) # 这个也很重要别忘了!!!


    max_red_blob = None # 存放最大红块
    while(True):

        if (pyb.millis() - time_start > delay):  #运行超过3秒,自动赋上一个角的坐标给色块当前坐标,并退出
            cx = x1
            cy = y1
            print("超时")
            break

        img = sensor.snapshot() # 拍张照

        for blob in img.find_blobs([thresholds[threshold_index]],pixels_threshold=pth, area_threshold=ath, merge=False):
            if blob.elongation()> blob_red_elo and blob.pixels() > max_red_size: # 筛选要圆的,而且还得是最大的
                max_red_blob = blob
                max_red_size = blob.pixels() # 遍历比较

                img.draw_cross(max_red_blob.cx(), max_red_blob.cy())

        if max_red_blob is not None: # 拍照直到有红色激光为止

            cx = max_red_blob.cx()   #注释掉莫名其妙就能用了
            cy = max_red_blob.cy()
            break # 退出while循环                             这里记得写超时部分

    print("---------------------------")




    #output_x.reset_I()
    #output_y.reset_I()
    if(abs(x1 - cx)  > 5  and abs(y1 - cy)  > 5): # 反馈,再获取一次色块坐标,取绝对值
        continue # 继续靠近
    else:
        break
print("已经追到嘞!")

time.sleep_ms(1000)

# ---------------------------------------------------------------------------------------

print("第五次:")

n = 0
#云台+1是左转  舵机+1是上转
time_start = pyb.millis()
while(True):

    err_x = x0 - cx   #两色点间x距离
    err_y = y0 - cy
    n = n + 1
    print("第几轮")
    print(n)
    print("起始点")
    print(cx)
    print(cy)
    print("目标点")
    print(x0)
    print(y0)
    print("当前误差")
    print(err_x)
    print(err_y)

    if (x0 - cx > 0): #在右边
        #当前角度 - 应该变化的距离, PID算法输出的误差值
        s1.angle(s1.angle() - abs(output_x.get_pid(err_x,scale_x))  ) #右转

    if (x0 - cx < 0): #在左边
        s1.angle(s1.angle() + abs(output_x.get_pid(err_x,scale_x))  )


#同理
    if (y0 - cy > 0): #在下面
        s2.angle(s2.angle() - abs(output_y.get_pid(err_y,scale_y))  )

    if (y0 - cy < 0): #在上面
        s2.angle(s2.angle() + abs(output_y.get_pid(err_y,scale_y))  )

    time.sleep_ms(200) # 这个也很重要别忘了!!!


    max_red_blob = None # 存放最大红块
    while(True):

        if (pyb.millis() - time_start > delay): #运行超过3秒,自动赋上一个角的坐标给色块当前坐标,并退出
            cx = x0
            cy = y0
            print("超时")
            break

        img = sensor.snapshot() # 拍张照

        for blob in img.find_blobs([thresholds[threshold_index]],pixels_threshold=pth, area_threshold=ath, merge=False):
            if blob.elongation()> blob_red_elo and blob.pixels() > max_red_size: # 筛选要圆的,而且还得是最大的
                max_red_blob = blob
                max_red_size = blob.pixels() # 遍历比较

                img.draw_cross(max_red_blob.cx(), max_red_blob.cy())

        if max_red_blob is not None: # 拍照直到有红色激光为止

            cx = max_red_blob.cx()   #注释掉莫名其妙就能用了
            cy = max_red_blob.cy()
            break # 退出while循环                             这里记得写超时部分

    print("---------------------------")




    #output_x.reset_I()
    #output_y.reset_I()
    if(abs(x0 - cx)  > 5  and abs(y0 - cy)  > 5): # 反馈,再获取一次色块坐标,取绝对值
        continue # 继续靠近
    else:
        break
print("已经追到嘞!")



time.sleep_ms(1000 * 60)
### --------------------------------一分钟后开始追踪绿色----------------------------------------------------------------------
#sensor.set_framesize(sensor.QQCIF)

#gx = 0
#gy = 0



#max_green_blob = None
#while(True):
    #if (pyb.millis() - time_start > delay): #运行超过3秒,自动赋上一个角的坐标给色块当前坐标,并退出
        #gx = x0
        #gy = y0
        #print("超时")
        #break
    #img = sensor.snapshot() # 拍张照

    #for blob in img.find_blobs([thresholds[1]],pixels_threshold=1, area_threshold=1, merge=False):
        #if blob.elongation()> 0 and blob.pixels() > 0: # 筛选要圆的,而且还得是最大的
            #max_green_blob = blob
            #max_green_size = blob.pixels() # 遍历比较

            #img.draw_cross(max_green_blob.cx(), max_green_blob.cy())

    #if max_green_blob is not None: # 拍照直到有绿色激光为止
        #gx = max_green_blob.cx()   #注释掉莫名其妙就能用了
        #gy = max_green_blob.cy()

        #break # 退出while循环





#time_start = pyb.millis()
#while(True):

    #err_x = gx - cx   #两色点间x距离
    #err_y = gy - cy
    #n = n + 1
    #print("第几轮")
    #print(n)
    #print("起始点")
    #print(cx)
    #print(cy)
    #print("目标点")
    #print(gx)
    #print(gy)
    #print("当前误差")
    #print(err_x)
    #print(err_y)

    #if (gx - cx > 0): #在右边
        ##当前角度 - 应该变化的距离, PID算法输出的误差值
        #s1.angle(s1.angle() - abs(output_x.get_pid(err_x,scale_x))  ) #右转

    #if (gx - cx < 0): #在左边
        #s1.angle(s1.angle() + abs(output_x.get_pid(err_x,scale_x))  )


##同理
    #if (gy - cy > 0): #在下面
        #s2.angle(s2.angle() - abs(output_y.get_pid(err_y,scale_y))  )

    #if (gy - cy < 0): #在上面
        #s2.angle(s2.angle() + abs(output_y.get_pid(err_y,scale_y))  )

    #time.sleep_ms(100) # 这个也很重要别忘了!!!


    #max_red_blob = None # 存放最大红块
    #while(True):

        #if (pyb.millis() - time_start > delay): #运行超过3秒,自动赋上一个角的坐标给色块当前坐标,并退出
            #cx = gx
            #cy = gy
            #print("超时")
            #break

        #img = sensor.snapshot() # 拍张照

        #for blob in img.find_blobs([thresholds[0]],pixels_threshold=pth, area_threshold=ath, merge=False):
            #if blob.elongation()> blob_red_elo and blob.pixels() > max_red_size: # 筛选要圆的,而且还得是最大的
                #max_red_blob = blob
                #max_red_size = blob.pixels() # 遍历比较

                #img.draw_cross(max_red_blob.cx(), max_red_blob.cy())

        #if max_red_blob is not None: # 拍照直到有红色激光为止

            #cx = max_red_blob.cx()   #注释掉莫名其妙就能用了
            #cy = max_red_blob.cy()

    #print("---------------------------")

    #time.sleep_ms(300)


    ##output_x.reset_I()
    ##output_y.reset_I()

#print("已经追到嘞!")



