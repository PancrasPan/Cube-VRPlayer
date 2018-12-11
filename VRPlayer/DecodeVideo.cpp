//#include"DecVideo.h"
//
//cv::Mat Mbuff[MaxLoadFrameBuffSize], m[MaxMatRemapBuffSize][6];//Ԥ����Ƶ֡loadframe ���� ��Ƶ֡matremap
//int Mbuff1 = 2;//Ԥ�����loadframe
//int Mbuff2 = 2;//��βloadframe
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
////��������ʼ��
//int x264_decoder_Init(AVCodecParserContext **parser, AVCodecContext **c, AVFrame **frame, AVPacket **pkt, uint8_t **inbuf)
//{
//
//	const AVCodec *codec;
//	//����������ݵ�buffer������buffer_size(����buffer_size=4096)
//	*inbuf = (uint8_t*)malloc(BUFFER_SIZE * sizeof(uint8_t));
//
//	//����x264�Ľ�����
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
//	//���ձ��������ͳ�ʼ������������
//	*parser = av_parser_init(codec->id);
//	if (!parser) {
//		fprintf(stderr, "parser not found\n");
//		exit(1);
//	}
//
//	//��msmpeg4�Լ�mpeg4���͵ı����������ڴ˴�������Ⱥ͸߶ȣ���Ϊ����bit���ﲻ���������ļ�
//	//��avcodec_alloc_context3֮��ʹ�ã���ʼ��AVCodeContext�Ա�ʹ��AVCodeContext
//	if (avcodec_open2(*c, codec, NULL) < 0) {
//		fprintf(stderr, "Could not open codec\n");
//		exit(1);
//	}
//
//	//������������ݰ��Լ�֡
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
////flush������
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
//		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)  //���뻹δ���
//			return;
//		else if (ret < 0) {
//			fprintf(stderr, "Error during decoding\n"); //�������
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
////�ͷŽ�����ռ���ڴ�
//void x264_decoder_Destroy(AVCodecContext **c, AVCodecParserContext **parser, AVFrame **frame, AVPacket **pkt, uint8_t **pic_inbuff)
//{
//	av_parser_close(*parser);
//	avcodec_free_context(c);
//	av_frame_free(frame);
//	av_packet_free(pkt);
//	free(*pic_inbuff);
//}
//
////�������� ���֡parser ���������c �����pkt �������֡frame 
//int x264_decodeVideo(AVCodecParserContext **parser, AVCodecContext **c, AVPacket **pkt, AVFrame *frame, uint8_t *pic_inbuff, int data_size)
//{
//	int ret;
//	uint8_t *data;
//	data = pic_inbuff;
//
//	if (data_size <= 0) {  //�������������Ϊ����������
//		return -1;
//	}
//	//ʹ��parser����������ݽ���Ϊ֡
//	while (data_size> 0) {
//		ret = av_parser_parse2(*parser, *c, &(*pkt)->data, &(*pkt)->size, data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
//		if (ret < 0) {
//			fprintf(stderr, "Error while parsing\n");
//			exit(1);
//		}
//		data += ret;   //����ָ�����
//		data_size -= ret; //��ǰʹ�õ�������
//#ifdef FRAME_INFO
//		printf_s("ret=%d,data_size=%d,pkt->size=%d\n", ret, data_size, (*pkt)->size);
//#endif
//		if ((*pkt)->size) { //һ֡�ָ����,����һ֡
//			ret = avcodec_send_packet(*c, *pkt);
//			if (ret < 0) {
//				fprintf(stderr, "Error sending a packet for decoding\n");
//				exit(1);
//			}
//			while (ret >= 0) {
//				ret = avcodec_receive_frame(*c, frame);
//				if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)  //���뻹δ���
//					break;
//				else if (ret < 0) {
//					fprintf(stderr, "Error during decoding\n"); //�������
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

