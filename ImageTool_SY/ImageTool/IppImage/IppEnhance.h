#pragma once
#include "..\IppImage.h"

void IppInverse(IppByteImage& img);//반전함수
void IppBrightness(IppByteImage& img, int n);//밝기 변환 함수
void IppContrast(IppByteImage& img, int n); //명암비 조절 함수


void IppGammaCorrection(IppByteImage& img, float gamma); //감마 보정 
void IppHistogram(IppByteImage& img, float histo[256]);
void IppHistogramStretching(IppByteImage& img);
void IppHistogramEqualizaiton(IppByteImage& img);
