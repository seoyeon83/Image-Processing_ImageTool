#pragma once

#include "..\IppImage.h"

// ����
void IppInverse(IppRgbImage& img);

//YUV ���� ��ȯ �Լ�
void RGB_TO_YUV(BYTE R, BYTE G, BYTE B, BYTE& Y, BYTE& U, BYTE& V);
void YUV_TO_RGB(BYTE Y, BYTE U, BYTE V, BYTE& R, BYTE& G, BYTE& B);

//YUV ���� ��� ����
void IppColorSplitYUV(IppRgbImage& imgColor, IppByteImage& imgY, IppByteImage& imgU, IppByteImage& imgV);

//Ʈ���÷� ���� ����
void IppColorEdge(IppRgbImage& imgSrc, IppByteImage& imgEdge);

//YUV���� ��� ��ġ��
bool IppColorCombineYUV(IppByteImage& imgY, IppByteImage& imgU, IppByteImage& imgV, IppRgbImage& imgColor);