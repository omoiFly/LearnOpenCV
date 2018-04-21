#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace cv;

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


//提高饱和度
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


int main() {
    // Origin Images
    auto src = imread("/home/lusx/Pictures/merge/street.jpg");
    resize(src, src, Size(src.cols/4, src.rows/4));
    auto cloud = imread("/home/lusx/Pictures/merge/cloud3.jpg");
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

    imwrite("before2.jpg", src);
    imwrite("after2.jpg", dst);

    destroyAllWindows();
    return 0;
}