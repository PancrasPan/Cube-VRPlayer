//#include"DecVideo.h"
//
//cv::Mat Mbuff[MaxLoadFrameBuffSize], m[MaxMatRemapBuffSize][6];//预存视频帧loadframe 处理 视频帧matremap
//int Mbuff1 = 2;//预存队首loadframe
//int Mbuff2 = 2;//队尾loadframe
//AVAudioFifo* audiofifo =NULL;//
//							 //
//cv::Mat avframe_to_cvmat(AVFrame *frame)
//{
//	AVFrame dst;
//	cv::Mat m;
//
//	memset(&dst, 0, sizeof(dst));
//
//	int w = frame->width, h = frame->height;
//	m = cv::Mat(h, w, CV_8UC3);
//	dst.data[0] = (uint8_t *)m.data;
//	avpicture_fill((AVPicture *)&dst, dst.data[0], AV_PIX_FMT_BGR24, w, h);
//
//	struct SwsContext *convert_ctx = NULL;
//	enum AVPixelFormat src_pixfmt = (enum AVPixelFormat)frame->format;
//	enum AVPixelFormat dst_pixfmt = AV_PIX_FMT_BGR24;
//	convert_ctx = sws_getContext(w, h, src_pixfmt, w, h, dst_pixfmt,
//		SWS_FAST_BILINEAR, NULL, NULL, NULL);
//	sws_scale(convert_ctx, frame->data, frame->linesize, 0, h,
//		dst.data, dst.linesize);
//	sws_freeContext(convert_ctx);
//
//	return m;
//}
////解码器初始化
//int x264_decoder_Init(AVCodecParserContext **parser, AVCodecContext **c, AVFrame **frame, AVPacket **pkt, uint8_t **inbuf)
//{
//
//	const AVCodec *codec;
//	//申请读入数据的buffer，计算buffer_size(假设buffer_size=4096)
//	*inbuf = (uint8_t*)malloc(BUFFER_SIZE * sizeof(uint8_t));
//
//	//查找x264的解码器
//	codec = avcodec_find_decoder(AV_CODEC_ID_H264);
//	if (!codec) {
//		fprintf(stderr, "Codec not found\n");
//		exit(1);
//	}
//	*c = avcodec_alloc_context3(codec);
//	if (!c) {
//		fprintf(stderr, "Could not allocate video codec context\n");
//		exit(1);
//	}
//	//按照编码器类型初始化解码器参数
//	*parser = av_parser_init(codec->id);
//	if (!parser) {
//		fprintf(stderr, "parser not found\n");
//		exit(1);
//	}
//
//	//对msmpeg4以及mpeg4类型的编码器必须在此处给出宽度和高度，因为编码bit流里不包含该类文件
//	//在avcodec_alloc_context3之后使用，初始化AVCodeContext以便使用AVCodeContext
//	if (avcodec_open2(*c, codec, NULL) < 0) {
//		fprintf(stderr, "Could not open codec\n");
//		exit(1);
//	}
//
//	//申请编码后的数据包以及帧
//	*pkt = av_packet_alloc();
//	if (!pkt)
//		exit(1);
//	*frame = av_frame_alloc();
//	if (!frame) {
//		fprintf(stderr, "Could not allocate video frame\n");
//		exit(1);
//	}
//	return 1;
//}
//
////flush缓冲区
//void x264_decoder_Flush(AVCodecContext **c, AVFrame **frame)
//{
//	int ret;
//	ret = avcodec_send_packet(*c, NULL);
//	if (ret < 0) {
//		fprintf(stderr, "Error sending a packet for decoding\n");
//		exit(1);
//	}
//
//	while (ret >= 0) {
//		ret = avcodec_receive_frame(*c, *frame);
//		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)  //解码还未完成
//			return;
//		else if (ret < 0) {
//			fprintf(stderr, "Error during decoding\n"); //解码出错
//			exit(1);
//		}
//#ifdef FRAME_INFO
//		printf("saving frame %3d,width=%d,height=%d\nFlush", (*c)->frame_number, (*c)->width, (*c)->height);
//#endif // FRAME_INFO
//		fflush(stdout);
//
//		/*cv::Mat yuvImg;
//		yuvImg.create(PIC_HEIGHT * 3 / 2, PIC_WIDTH, CV_8UC1);
//		memcpy(yuvImg.data, (*frame)->data[0], PIC_SIZE * sizeof(unsigned char));
//		memcpy(yuvImg.data + PIC_SIZE, (*frame)->data[1], PIC_SIZE / 4 * sizeof(unsigned char));
//		memcpy(yuvImg.data + PIC_SIZE * 5 / 4, (*frame)->data[2], PIC_SIZE / 4 * sizeof(unsigned char));
//		cv::Mat rgbImg, tmp;
//		cv::cvtColor(yuvImg, rgbImg, CV_YUV2BGR_I420);*/
//
//
//		if ((Mbuff1 + 1) % MaxLoadFrameBuffSize != Mbuff2) {
//			Mbuff[Mbuff1] = avframe_to_cvmat(*frame);
//
//	
//			//Mbuff[Mbuff1] = rgbImg;
//			Mbuff1 = (Mbuff1 + 1) % MaxLoadFrameBuffSize;
//		}
//	}
//}
////释放解码器占用内存
//void x264_decoder_Destroy(AVCodecContext **c, AVCodecParserContext **parser, AVFrame **frame, AVPacket **pkt, uint8_t **pic_inbuff)
//{
//	av_parser_close(*parser);
//	avcodec_free_context(c);
//	av_frame_free(frame);
//	av_packet_free(pkt);
//	free(*pic_inbuff);
//}
//
////从左至右 拆分帧parser 编码控制器c 输入包pkt 输出解码帧frame 
//int x264_decodeVideo(AVCodecParserContext **parser, AVCodecContext **c, AVPacket **pkt, AVFrame *frame, uint8_t *pic_inbuff, int data_size)
//{
//	int ret;
//	uint8_t *data;
//	data = pic_inbuff;
//
//	if (data_size <= 0) {  //如果解码数据量为不是正整数
//		return -1;
//	}
//	//使用parser将读入的数据解析为帧
//	while (data_size> 0) {
//		ret = av_parser_parse2(*parser, *c, &(*pkt)->data, &(*pkt)->size, data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
//		if (ret < 0) {
//			fprintf(stderr, "Error while parsing\n");
//			exit(1);
//		}
//		data += ret;   //数据指针后移
//		data_size -= ret; //当前使用的数据量
//#ifdef FRAME_INFO
//		printf_s("ret=%d,data_size=%d,pkt->size=%d\n", ret, data_size, (*pkt)->size);
//#endif
//		if ((*pkt)->size) { //一帧分割完成,解码一帧
//			ret = avcodec_send_packet(*c, *pkt);
//			if (ret < 0) {
//				fprintf(stderr, "Error sending a packet for decoding\n");
//				exit(1);
//			}
//			while (ret >= 0) {
//				ret = avcodec_receive_frame(*c, frame);
//				if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)  //解码还未完成
//					break;
//				else if (ret < 0) {
//					fprintf(stderr, "Error during decoding\n"); //解码出错
//					exit(1);
//				}
//#ifdef FRAME_INFO
//				printf("saving frame %3d,width=%d,height=%dDecode\n", (*c)->frame_number, frame->width, frame->height);
//#endif // FRAME_INFO
//				fflush(stdout);
//
//				/*cv::Mat yuvImg;
//				yuvImg.create(PIC_HEIGHT * 3 / 2, PIC_WIDTH, CV_8UC1);
//				memcpy(yuvImg.data, frame->data[0], PIC_SIZE * sizeof(unsigned char));
//				memcpy(yuvImg.data + PIC_SIZE, frame->data[1], PIC_SIZE/4 * sizeof(unsigned char));
//				memcpy(yuvImg.data + PIC_SIZE * 5 / 4, frame->data[2], PIC_SIZE/4 * sizeof(unsigned char));
//				cv::Mat rgbImg, tmp;
//				cv::cvtColor(yuvImg, rgbImg, CV_YUV2BGR_I420);*/
//				
//				if ((Mbuff1 + 1) % MaxLoadFrameBuffSize != Mbuff2) {
//					Mbuff[Mbuff1] = avframe_to_cvmat(frame);
//					//Mbuff[Mbuff1] = rgbImg;
//					Mbuff1 = (Mbuff1 + 1) % MaxLoadFrameBuffSize;
//				}
//
//			}
//		}
//	}
//	return 0;
//}

