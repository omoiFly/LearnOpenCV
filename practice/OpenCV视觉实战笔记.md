# OpenCV 图像处理实战

### 你的名字滤镜

一共用到了三个部分，分别为:

- 背景前景分割

- 和事先准备的云朵图片进行融合

- 对图像饱和度进行加强(增强动漫感)

主函数如下：

```c++
int main() {
    // Origin Images
    auto src = imread("/home/lusx/Pictures/merge/street.jpg");
    resize(src, src, Size(src.cols/3, src.rows/3));
    auto cloud = imread("/home/lusx/Pictures/merge/cloud3.jpg");
    resize(cloud, cloud, src.size());
    // Temp Variables
    Mat hsv_image;
    vector<Mat> planes;
    // Separate to HSV
    cvtColor(src, hsv_image, COLOR_BGR2HSV);
    split(hsv_image, planes);
    // Equalization Value(亮度)
    equalizeHist(planes[2], planes[2]);
    merge(planes, hsv_image);
    inRange(hsv_image, Scalar(100, 43, 46), Scalar(124, 255, 255), hsv_image);
    // inrange的区间设计不合理，太宽了
    erode(hsv_image, hsv_image, Mat());
    dilate(hsv_image, hsv_image, Mat());
    auto mask = hsv_image.clone();
    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////

    auto max_contour = find_maximum_contour(mask);
    auto max_rect = boundingRect(max_contour);
    if (max_rect.height == 0 || max_rect.width == 0) {
        max_rect = Rect(0, 0, mask.cols, mask.rows);
    }
    auto mat_dst = src.clone();
    mask = mask(max_rect);
    resize(cloud, cloud, max_rect.size());
    auto center = Point((max_rect.x + max_rect.width)/2, (max_rect.y + max_rect.height)/2);
    Mat clone;
    seamlessClone(cloud, src, mask, center, clone, NORMAL_CLONE);
    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////

    // Enhance Saturation
    bilateralFilter(clone, hsv_image, 5, 10.0, 2.0);
    cvtColor(hsv_image, hsv_image, COLOR_BGR2YCrCb);
    split(hsv_image, planes);
    equalizeHist(planes[0], planes[0]);
    merge(planes, hsv_image);
    cvtColor(hsv_image, hsv_image, COLOR_YCrCb2BGR);
    auto dst = EnhanceSaturation(hsv_image);

    imshow("before clone", src);
    imshow("after clone", dst);

    waitKey();
    destroyAllWindows();
    return 0;
}
```

其中用到了两个函数，分别实现如下：

```c++
vector<Point> find_maximum_contour(Mat& src) {
    int max_contours = -1;
    vector<Point> max_contour;
    vector<vector<Point>> contours;
    findContours(src, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    for (const auto &contour : contours) {
        auto tmp_area = contourArea(contour);
        if (max_contours < tmp_area) {
            max_contour = contour;
        }
    }
    return max_contour;
}
```

```c++
Mat EnhanceSaturation(Mat& temp)
{
    Mat matDst;
    Mat Img_out(temp.size(), CV_32FC3);
    temp.convertTo(Img_out, CV_32FC3);
    Mat Img_in(temp.size(), CV_32FC3);
    temp.convertTo(Img_in, CV_32FC3);
    // define the iterator of the input image
    MatIterator_<Vec3f> inp_begin, inp_end;
    inp_begin=Img_in.begin<Vec3f>();
    inp_end =Img_in.end<Vec3f>();
    // define the iterator of the output image
    MatIterator_<Vec3f> out_begin, out_end;
    out_begin=Img_out.begin<Vec3f>();
    out_end =Img_out.end<Vec3f>();
    // increment (-100.0, 100.0)
    float Increment=50.0/100.0;   //饱和度参数调整
    float delta=0;
    float minVal, maxVal;
    float t1, t2, t3;
    float L,S;
    float alpha;

    for(; inp_begin!=inp_end; inp_begin++, out_begin++)
    {
        t1=(*inp_begin)[0];
        t2=(*inp_begin)[1];
        t3=(*inp_begin)[2];

        minVal=std::min(std::min(t1,t2),t3);
        maxVal=std::max(std::max(t1,t2),t3);
        delta=(maxVal-minVal)/255.0;
        L=0.5*(maxVal+minVal)/255.0;
        S=std::max(0.5*delta/L, 0.5*delta/(1-L));

        if (Increment>0)
        {
            alpha=max(S, 1-Increment);
            alpha=1.0/alpha-1;
            (*out_begin)[0]=(*inp_begin)[0]+((*inp_begin)[0]-L*255.0)*alpha;
            (*out_begin)[1]=(*inp_begin)[1]+((*inp_begin)[1]-L*255.0)*alpha;
            (*out_begin)[2]=(*inp_begin)[2]+((*inp_begin)[2]-L*255.0)*alpha;
        }
        else
        {
            alpha=Increment;
            (*out_begin)[0]=L*255.0+((*inp_begin)[0]-L*255.0)*(1+alpha);
            (*out_begin)[1]=L*255.0+((*inp_begin)[1]-L*255.0)*(1+alpha);
            (*out_begin)[2]=L*255.0+((*inp_begin)[2]-L*255.0)*(1+alpha);

        }
    }
    Img_out /=255;
    Img_out.convertTo(matDst,CV_8UC3,255);

    return matDst;
}
```

使用Python3.6 进行了重写，实现如下：

```python
def main():
    src = cv.imread("/home/lusx/Pictures/merge/street.jpg")
    src = cv.resize(src, None, fx=0.25, fy=0.25)
    mask = find_mask(src)
    max_contour = find_maximum_contour(mask)
    cv.imshow("src", src)
    cv.imshow("mask", mask)
    cv.waitKey()
    cv.destroyAllWindows()
```

函数实现如下：

```python
def find_maximum_contour(src):
    max_contours = []
    max_contours_area = -1
    image, contours, hierarchy = cv.findContours(src, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv.contourArea(contour)
        if area > max_contours_area:
            max_contours = contour
            max_contours_area = area
    return max_contours
```

```python
def find_mask(src):
    hsv_image = cv.cvtColor(src, cv.COLOR_BGR2HSV)
    planes = cv.split(hsv_image)
    cv.equalizeHist(planes[2], planes[2])
    cv.merge(planes, hsv_image)
    hsv_image = cv.inRange(hsv_image, (100, 43, 46), (124, 255, 255))
    cv.erode(hsv_image, None, hsv_image)
    cv.dilate(hsv_image, None, hsv_image)
    return hsv_image
```

```python
def clone(src, mask):
    cloud = cv.imread("/home/lusx/Pictures/merge/cloud3.jpg")
    max_contour = find_maximum_contour(mask)
    x, y, width, height = cv.boundingRect(max_contour)
    if x is None:
        x = 0
        y = 0
        width = mask.shape[0]
        height = mask.shape[1]
    roi = mask[y:height, x:width]
    center = (int((width - x)/2), int((height - y)/2))
    cloud = cv.resize(cloud, (roi.shape[1], roi.shape[0]))

    dst = cv.seamlessClone(cloud, src, roi, center, cv.NORMAL_CLONE)
    return dst
  
```

```python
def enhance_saturation(img):

    return img
```

```python
def main():
    src = cv.imread("/home/lusx/Pictures/merge/street.jpg")
    if src.shape[0] > 3000:
        src = cv.resize(src, None, fx=0.25, fy=0.25)
    elif src.shape[0] > 1500:
        src = cv.resize(src, None, fx=0.5, fy=0.5)
    else:
        pass
    mask = find_mask(src)
    dst = clone(src, mask)
    dst = enhance_saturation(dst)

    cv.imshow("src", src)
    cv.imshow("dst", dst)
    cv.waitKey()
    cv.destroyAllWindows()
```

##### 对项目的总结

1. 对膨胀腐蚀操作的解释：

   膨胀: 白->黑 (白色部分向黑色部分扩展) dilate

   腐蚀: 黑->白 (黑色部分向白色部分扩展) erode

2. CV_8U 0~255

   CV_32F 0~1

3. mask 掩膜操作

   类似ROI

   但是不局限于Rect

   待处理部分填255(白)

   不处理部分填0(黑)

4. seamlessClone 泊松融合， 在后面的思考中详述，也是实现滤镜的重点

5. HSV 颜色空间的处理

   - 提取出图像中蓝色的部分(蓝天)，对前景和背景进行分离操作

6. 图像饱和度的增强(使用了查表法进行)，原理在后面的思考中详述

##### 思考

- 对于二阶泊松融合的理解

  函数的梯度即函数的变化趋势。对图像融合来说，打个比方，如果要融合两个函数(图像)，可以记下要融合的图片的变化趋势(梯度)，让原图按照同样的趋势“生长”出要融合的部分，图像的衔接就会更加自然。

  函数的一阶导是函数的变化趋势(斜率)，那么二阶导就是变化趋势的趋势，如果我们能够保证原图和新图的变化趋势的趋势(二阶导)差距达到最小，那么它们的趋势变化也会达到最小，这样函数就不会有突兀的改变。上面的这段描述最后在Poission Image Editing 这篇论文中最后变为解如下的函数：

$$\mathop{\min}\limits_{f}\iint_Ω\left|\nabla f-{\bf v}\right|^2 with \left. f \right| _{\partialΩ}=\left.f^*\right|_{\partialΩ}$$

其中f就是最后融合的图像，f*是原来的图像，v就是要融合的图像，通过一系列的迭代优化技术，最后获得融合的结果图像f。

*在融合过程中，其实并没有将要融合的图片“糊”上去，而是通过计算散度(也就是上文提到的二阶导，只不过这里是偏导)，让原图的待融合部分按照上式生成的规则算出这部分的像素值，最后获得融合后的图片*

- 图像饱和度的增强

  可以达到动漫中的效果，通过一个映射表的映射关系将特定的RGB映射到另一个值，且效果较好
