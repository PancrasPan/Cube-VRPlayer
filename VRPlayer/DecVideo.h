#pragma once

#include <stdio.h>
#include <WinSock2.h>
#include <windows.h>
#include<time.h>
#include <WS2tcpip.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavformat/avformat.h>
#include <libavutil/audio_fifo.h>
#include <libswscale/swscale.h>
#include "libswresample/swresample.h"
}

#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#if defined( OSX )
#include <Foundation/Foundation.h>
#include <AppKit/AppKit.h>
#include <OpenGL/glu.h>
// Apple's version of glut.h #undef's APIENTRY, redefine it
#define APIENTRY
#else
#include <GL/glu.h>
#endif
#include <stdio.h>
#include <string>
#include <cstdlib>
#include "SDL_thread.h"
#include "openvr.h"
#include "lodepng.h"
#include "Matrices.h"
#include "pathtools.h"
#if defined(POSIX)
#include "unistd.h"
#endif
#ifndef _WIN32
#define APIENTRY
#endif
#ifndef _countof
#define _countof(x) (sizeof(x)/sizeof((x)[0]))
#endif

#include <GLTools.h>	// OpenGL toolkit
#include <GLMatrixStack.h>
#include <GLFrame.h>
#include <GLFrustum.h>
#include <GLGeometryTransform.h>
#include <StopWatch.h>
#include <opencv2/cudawarping.hpp>
#include <math.h>
#include <stdlib.h>
#include <opencv2\core\opengl.hpp>
#include <GL/glut.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#pragma comment(lib,"ws2_32.lib")
#define PORT_NUMBER 8088
#define BUFFER_SIZE 4096
#define PIC_SIZE 2097152
//#define BUFFER_SIZE 8192
//#define PIC_SIZE 8192000
#define SERVER_IP "100.64.171.233" 
//#define PIC_HEIGHT 1920   //qqq.h264
//#define PIC_WIDTH 3840
//#define PIC_HEIGHT 1024  //2k
//#define PIC_WIDTH 2048
#define PIC_HEIGHT 1024   //out.h264
#define PIC_WIDTH 2048
#define YUV_BUF PIC_HEIGHT*PIC_WIDTH*1.5     //w*h*1.5
//调试选项
//#define DEBUG    //读写文件
//#define FRAME_INFO //每帧输出信息
#define MaxMatRemapBuffSize 100
#define MaxLoadFrameBuffSize 100

//初始化socket操作
int client_transfer_Init(WSAData *wsaData, SOCKET *sockfd);
//设置非阻塞方式
int set_non_Block(SOCKET socket);
//非阻塞方式发送数据
int send_non_Block(SOCKET socket, char *buffer, int length, int flags);
//非阻塞方式接收数据
int recv_non_Block(SOCKET socket, char *buffer, int length, int flags);
//非阻塞方式连接
int connect_non_Block(SOCKET socket, const struct sockaddr *address, int address_len);
//销毁客户端传输socket
void client_transfer_Destroy(SOCKET *socket);

//解码器初始化
int x264_decoder_Init(AVCodecParserContext **parser, AVCodecContext **c, AVFrame **frame, AVPacket **pkt, uint8_t **inbuf);
//释放解码器占用内存
void x264_decoder_Destroy(AVCodecContext **c, AVCodecParserContext **parser, AVFrame **frame, AVPacket **pkt,uint8_t **pic_outbuf);
//输入一帧进行解码
int x264_decodeVideo(AVCodecParserContext **parser, AVCodecContext **c, AVPacket **pkt, AVFrame *frame, uint8_t *pic_inbuff,int data_size);
//flush缓冲区
void x264_decoder_Flush(AVCodecContext **c, AVFrame **frame);

