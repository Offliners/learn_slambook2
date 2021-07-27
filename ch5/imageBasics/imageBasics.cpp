#include<iostream>
#include<chrono>

using namespace std;

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    // 讀取argv[1]指定的圖像
    cv::Mat image;
    image = cv::imread(argv[1]); //cv::imread函數讀取指定路徑下的圖像

    // 判斷圖像文件是否正確讀取
    if (image.data == nullptr) //數據不存在,可能是文件不存在
    { 
        cerr << "文件" << argv[1] << "不存在." << endl;
        return 0;
    }

    // 文件順利讀取, 首先输出一些基本訊息
    cout << "圖像寬為" << image.cols << ", 高為" << image.rows << ", 通道數為" << image.channels() << endl;
    cv::imshow("image", image);      // 用cv::imshow顯示圖像
    cv::waitKey(0);                  // 暫停程序,等待一個按鍵輸入

    // 判斷image的類型
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) // 圖像類型不符合要求
    {
        cout << "請输入一張彩色圖或灰階圖." << endl;
        return 0;
    }

    // 遍歷圖像, 請注意以下遍歷方式亦可使用於隨機像素訪問
    // 使用 std::chrono 來給算法計時
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; ++y) 
    {
        // 用cv::Mat::ptr獲得圖像的行指標
        unsigned char *row_ptr = image.ptr<unsigned char>(y);  // row_ptr是第y行的頭指標
        for (size_t x = 0; x < image.cols; ++x) 
        {
            // 訪問位於 x,y 處的像素
            unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 指向待訪問的像素數據
            // 輸出該像素的每個通道, 如果是灰階圖就只有一個通道
            for (int c = 0; c != image.channels(); ++c)
                unsigned char data = data_ptr[c]; // data為I(x,y)第c個通道的值
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
    cout << "遍歷圖像用時：" << time_used.count() << " 秒。" << endl;

    // 關於 cv::Mat 的拷貝
    // 直接賦值並不會拷貝數據
    cv::Mat image_another = image;
    // 修改 image_another 會導致 image 發生變化
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 將左上角100*100的塊置零
    cv::imshow("image", image);
    cv::waitKey(0);

    // 使用clone函數來拷貝數據
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    // 對於圖像還有很多基本的操作,如剪切, 旋轉, 縮放等, 限於篇幅就不一一介紹了,請參考OpenCV官方文黨查詢每個函數的調用方法.
    cv::destroyAllWindows();
    return 0;
}
