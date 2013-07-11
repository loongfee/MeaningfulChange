#include "GdalRasterApp.h"
#include <string>
#include <iostream>
#include <vector>
using namespace std;

int meaningful_change(string cd_map_file, string result_file)
{
	GdalRasterApp cd_map;
	cd_map.open(cd_map_file.c_str());
	int iBandCount = cd_map.nBand();
	int iTileCountX = cd_map.getTileCountX();
	int iTileCountY = cd_map.getTileCountY();
	int iWidth = cd_map.width();
	int iHeight = cd_map.height();

	GDALDriver *poDriver;	//驱动，用于创建新的文件
	poDriver=GetGDALDriverManager()->GetDriverByName("GTIFF");
	char **papszMetadata = poDriver->GetMetadata();//获取格式类型
	GDALDataset *poDatasetNew;
	// 输出栅格
	poDatasetNew = poDriver->Create(result_file.c_str(), iWidth, iHeight, 1, GDT_Byte, papszMetadata);//根据文件路径文件名，图像宽，高，波段数，数据类型，文件类型，创建新的数据集				
	poDatasetNew->SetProjection(cd_map.getGetProjectionRef());
	poDatasetNew->SetGeoTransform(cd_map.getGeoTransform());//坐标赋值,与全色相同

	int nCount = 0;
	double norm_threshold = 20.0;
	int bgColor = 0;
	int fgColor = 255;
	for (int i = 0;i < iTileCountX;++i)
	{
		for (int j = 0;j < iTileCountY;++j)
		{
			GdalRasterApp::RasterBuf *pBuf = cd_map.getTileData(i, j, iBandCount);
			int bufWidth = pBuf->iBufWidth;
			int bufHeight = pBuf->iBufHeight;
			int bufBand = pBuf->iBandCount;
			int offsetX, offsetY;
			cd_map.getTileOffset(i, j, offsetX, offsetY);
			cv::Mat change_image(cv::Size(bufWidth, bufHeight), CV_8UC1, cv::Scalar(fgColor));

			//cv::Mat lbp_change(cv::Size(bufWidth, bufHeight), CV_8UC1);
			cv::Mat lbp_change = img_int2byte_band(pBuf, 0);
			//cv::Mat lbp_change = img_float2byte_band(pBuf, 0);
			//memcpy(lbp_change.data, pBuf->data, bufWidth*bufHeight);
			//cv::imwrite("E:\\minus.tif", lbp_change);
			int lbp_change_threshold = cvThresholdOtsu(lbp_change);

			int step = change_image.step;
			// change detection
			for (int row = 0; row < bufHeight; ++row) {
				for (int col = 0; col < bufWidth; ++col) {
					int lbp_change_ = lbp_change.data[row*lbp_change.step+col];
					// 判断是否变化很小
					if (lbp_change_ < lbp_change_threshold)
					{
						change_image.data[row*step+col] = bgColor;
						continue;
					}

					std::vector<double> change_vector(pFeatureBuf->iBandCount);
					for (int k = 0;k < pFeatureBuf->iBandCount;++k)
					{
						int pos = row * bufWidth * pFeatureBuf->iBandCount + col*pFeatureBuf->iBandCount + k;
						change_vector[k] = ((float*)pFeatureBuf->data)[pos];
					}

					//如果变化是否和样本相似
					for (int m = 0; m < (int)samples.size(); m++)
					{
						double angle_ = samples[m][0] - change_vector[0];
						double norm_ = samples[m][1] - change_vector[1];
						//double similarity = VectorSimilarity(samples[m], change_vector);
						//double distance = VectorDistance(samples[m], change_vector);
						//double angle = VectorAngle(samples[m], change_vector);
						//if (fabs(similarity) > similarityThreshold && distance < 15.0)
						//if (fabs(angle) < 10.0 && distance < 20.0)
						//if (fabs(similarity) < similarityThreshold)
						if (fabs(angle_) < similarityThreshold && fabs(norm_) < 10.0)
							//if (fabs(norm) < 10.0)
						{
							change_image.data[row*step+col] = bgColor;
							break;
						}
					}
				}
			}

			//RasterBuf2Opencv(pBuf, change_image);
			//cv::Mat change_image(bufHeight, bufWidth, CV_8U(bufBand));
			//int nBandDataSize = GDALGetDataTypeSize( pBuf->eDataType ) / 8;
			//memcpy(change_image.data, (GByte*)(pBuf->data), bufWidth*bufHeight*bufBand*nBandDataSize);
			//cvtColor( change_image, change_image, CV_BGR2GRAY );
			//int threshold = cvThresholdOtsu(change_image);
			//threshold = 60;
			//cv::threshold( change_image, change_image, threshold, 255, CV_THRESH_BINARY);
			//cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
			//cv::morphologyEx(change_image, change_image, cv::MORPH_CLOSE, element);
			//element = cv::getStructuringElement(cv::MORPH_CROSS , cv::Size(7, 7));
			//cv::morphologyEx(change_image, change_image, cv::MORPH_OPEN, element);
			//element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
			//cv::morphologyEx(change_image, change_image, cv::MORPH_OPEN, element);
			//cv::GaussianBlur(change_image, change_image, cv::Size(5,5), 1.5);
			//cv::morphologyEx(change_image, change_image, cv::MORPH_CLOSE, element);

			//cv::imwrite("E:\\test0.tif", change_image);
			cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2,2));
			//cv::morphologyEx(change_image, change_image, cv::MORPH_CLOSE, element);
			int nTemplate = sqrt(smallArea);
			nTemplate = nTemplate / 2 * 2 + 1;
			cv::medianBlur(change_image, change_image, nTemplate);
			//cv::imwrite("E:\\test1.tif", change_image);
			// 先闭运算
			nTemplate = nTemplate / 2;
			element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
			cv::morphologyEx(change_image, change_image, cv::MORPH_CLOSE, element);
			// 再开运算
			//element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
			//cv::morphologyEx(change_image, change_image, cv::MORPH_OPEN, element);
			//cv::imwrite("E:\\test2.tif", change_image);
			BinaryRemoveSmall(change_image, change_image, smallArea, bgColor, fgColor);
			//cv::Mat tmpMat = change_image.clone();
			////cv::GaussianBlur(tmpMat, tmpMat, cv::Size(5,5), 1.5);
			//std::vector<std::vector<cv::Point> > contours;
			//std::vector<cv::Vec4i> hierarchy;
			//cv::findContours(tmpMat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			////for (std::vector<std::vector<cv::Point> >::iterator iter_Contours = contours.begin(); iter_Contours != contours.end();++iter_Contours)
			//int nContours = (int)contours.size();
			//for (int iContours = 0; iContours < nContours;++iContours)
			//{
			//	std::vector<cv::Point> contour = contours[iContours];
			//	double tempArea = fabs(cv::contourArea(contour));
			//	cv::Rect rect = cv::boundingRect(contour);
			//	//当连通域的中心点为白色，而且面积较小时，用黑色进行填充 
			//	if (tempArea < miniArea)
			//	{
			//		int pos_center = step*(rect.y+rect.height/2)+rect.x+rect.width/2;
			//		//if (255 == change_image.data[pos_center])
			//		{
			//			for(int y = rect.y;y<rect.y+rect.height;y++)
			//			{
			//				for(int x =rect.x;x<rect.x+rect.width;x++) 
			//				{
			//					int pos_ = y*step+x;
			//					if(255 == change_image.data[pos_])
			//					{
			//						change_image.data[pos_] = 0;
			//					}
			//				}
			//			}
			//		}
			//	}
			//}

			//cv::imwrite("E:\\test3.tif", change_image);
			////cv::imwrite("E:\\test.tif", change_image);

			if (_changeOutType::raster == outType)
			{
				// 输出栅格
				CPLErr gdal_err = poDatasetNew->RasterIO(GF_Write, offsetX, offsetY, bufWidth, bufHeight, change_image.data, bufWidth, bufHeight,\
					GDT_Byte, 1, 0, 0, 0, 0);
			}
			else if (_changeOutType::vector == outType)
			{
				// 输出矢量
				std::vector<std::vector<cv::Point> > contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(change_image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
				addContour2ShapeArea(contours, out_layer_file, raster_minus.getGeoTransform(), raster_minus.getGetProjectionRef(), nCount, raster_minus.getBufInfo()->iBufOffsetX, raster_minus.getBufInfo()->iBufOffsetY);
				nCount += (int)contours.size();
			}
			pBuf = NULL;
		}
	}
	raster_minus.close();
	raster_feature.close();
	poDatasetNew->FlushCache();
	GDALClose(poDatasetNew);
}

void main()
{
	
}