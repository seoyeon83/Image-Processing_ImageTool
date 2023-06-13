#pragma once

#include "..\IppImage.h"

// 반전
void IppInverse(IppRgbImage& img);

//YUV 색모델 변환 함수
void RGB_TO_YUV(BYTE R, BYTE G, BYTE B, BYTE& Y, BYTE& U, BYTE& V);
void YUV_TO_RGB(BYTE Y, BYTE U, BYTE V, BYTE& R, BYTE& G, BYTE& B);

//YUV 색상 평면 분할
void IppColorSplitYUV(IppRgbImage& imgColor, IppByteImage& imgY, IppByteImage& imgU, IppByteImage& imgV);

//트루컬러 엣지 검출
void IppColorEdge(IppRgbImage& imgSrc, IppByteImage& imgEdge);

//YUV색상 평면 합치기
bool IppColorCombineYUV(IppByteImage& imgY, IppByteImage& imgU, IppByteImage& imgV, IppRgbImage& imgColor);