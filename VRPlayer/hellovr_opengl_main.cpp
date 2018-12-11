#include"DecVideo.h"
using namespace cv;
using namespace std;
void ThreadSleep(unsigned long nMilliseconds)
{
#if defined(_WIN32)
	::Sleep(nMilliseconds);
#elif defined(POSIX)
	usleep(nMilliseconds * 1000);
#endif
}
class CGLRenderModel
{
public:
	CGLRenderModel(const std::string & sRenderModelName);
	~CGLRenderModel();
	bool BInit(const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture);
	void Cleanup();
	void Draw();
	const std::string & GetName() const { return m_sModelName; }
private:
	GLuint m_glVertBuffer;
	GLuint m_glIndexBuffer;
	GLuint m_glVertArray;
	GLuint m_glTexture;
	GLsizei m_unVertexCount;
	std::string m_sModelName;
};
static bool g_bPrintf = true;
class CMainApplication
{
public:
	CMainApplication(int argc, char *argv[]);
	virtual ~CMainApplication();
	bool BInit();
	bool BInitGL();
	bool BInitCompositor();
	void SetupRenderModels();
	void Shutdown();
	void RunMainLoop();
	bool HandleInput();
	void ProcessVREvent(const vr::VREvent_t & event);
	void RenderFrame();
	bool SetupTexturemaps();
	void SetupScene();
	void AddCubeToScene(Matrix4 mat, std::vector<float> &vertdata);
	void AddCubeVertex(float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata);
	void RenderControllerAxes();
	bool SetupStereoRenderTargets();
	void SetupCompanionWindow();
	void SetupCameras();
	void RenderStereoTargets();
	void RenderCompanionWindow();
	void RenderScene(vr::Hmd_Eye nEye);
	Matrix4 GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye);
	Matrix4 GetHMDMatrixPoseEye(vr::Hmd_Eye nEye);
	Matrix4 GetCurrentViewProjectionMatrix(vr::Hmd_Eye nEye);
	void UpdateHMDMatrixPose();
	Matrix4 ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t &matPose);
	GLuint CompileGLShader(const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader);
	bool CreateAllShaders();
	void SetupRenderModelForTrackedDevice(vr::TrackedDeviceIndex_t unTrackedDeviceIndex);
	CGLRenderModel *FindOrLoadRenderModel(const char *pchRenderModelName);
private:
	bool m_bDebugOpenGL;
	bool m_bVerbose;
	bool m_bPerf;
	bool m_bVblank;
	bool m_bGlFinishHack;
	vr::IVRSystem *m_pHMD;
	vr::IVRRenderModels *m_pRenderModels;
	std::string m_strDriver;
	std::string m_strDisplay;
	vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];
	bool m_rbShowTrackedDevice[vr::k_unMaxTrackedDeviceCount];
private: // SDL bookkeeping 薄记
	SDL_Window * m_pCompanionWindow;
	uint32_t m_nCompanionWindowWidth;
	uint32_t m_nCompanionWindowHeight;
	SDL_GLContext m_pContext;
private: // OpenGL bookkeeping
	int m_iTrackedControllerCount;
	int m_iTrackedControllerCount_Last;
	int m_iValidPoseCount;
	int m_iValidPoseCount_Last;
	bool m_bShowCubes;//显示屏是否显示
	std::string m_strPoseClasses;                            // what classes we saw poses for this frame
	char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount];   // for each device, a character representing its class
	int m_iSceneVolumeWidth;
	int m_iSceneVolumeHeight;
	int m_iSceneVolumeDepth;
	float m_fScaleSpacing;
	float m_fScale;
	int m_iSceneVolumeInit;                                  // if you want something other than the default 20x20x20
	float m_fNearClip;
	float m_fFarClip;
	GLuint m_iTexture;
	unsigned int m_uiVertcount;
	GLuint m_glSceneVertBuffer;
	GLuint m_unSceneVAO;
	GLuint m_unCompanionWindowVAO;
	GLuint m_glCompanionWindowIDVertBuffer;
	GLuint m_glCompanionWindowIDIndexBuffer;
	unsigned int m_uiCompanionWindowIndexSize;
	GLuint m_glControllerVertBuffer;
	GLuint m_unControllerVAO;
	unsigned int m_uiControllerVertcount;
	Matrix4 m_mat4HMDPose;
	Matrix4 m_mat4eyePosLeft;
	Matrix4 m_mat4eyePosRight;
	Matrix4 m_mat4ProjectionCenter;
	Matrix4 m_mat4ProjectionLeft;
	Matrix4 m_mat4ProjectionRight;
	struct VertexDataScene
	{
		Vector3 position;
		Vector2 texCoord;
	};
	struct VertexDataWindow
	{
		Vector2 position;
		Vector2 texCoord;
		VertexDataWindow(const Vector2 & pos, const Vector2 tex) : position(pos), texCoord(tex) {	}
	};
	GLuint m_unSceneProgramID;
	GLuint m_unCompanionWindowProgramID;
	GLuint m_unControllerTransformProgramID;
	GLuint m_unRenderModelProgramID;
	GLint m_nSceneMatrixLocation;
	GLint m_nControllerMatrixLocation;
	GLint m_nRenderModelMatrixLocation;
	struct FramebufferDesc
	{
		GLuint m_nDepthBufferId;
		GLuint m_nRenderTextureId;
		GLuint m_nRenderFramebufferId;
		GLuint m_nResolveTextureId;
		GLuint m_nResolveFramebufferId;
	};
	FramebufferDesc leftEyeDesc;
	FramebufferDesc rightEyeDesc;
	bool CreateFrameBuffer(int nWidth, int nHeight, FramebufferDesc &framebufferDesc);
	uint32_t m_nRenderWidth;
	uint32_t m_nRenderHeight;
	std::vector< CGLRenderModel * > m_vecRenderModels;
	CGLRenderModel *m_rTrackedDeviceToRenderModel[vr::k_unMaxTrackedDeviceCount];
};
/************全局变量******************/
CvCapture * capture = NULL;
char * VideoAddr = "全景视频-测试.mp4";
Mat mm, M, warpMat1[6], warpMat2[6], dst[6], warpMat[6];

cv::Mat Mbuff[MaxLoadFrameBuffSize], m[MaxMatRemapBuffSize][6];//预存视频帧loadframe 处理 视频帧matremap
int Mbuff1=2;//预存队首loadframe
int Mbuff2=2;//队尾loadframe
AVAudioFifo* audiofifo=NULL;//
//cv::Mat Mbuff[MaxLoadFrameBuffSize], m[MaxMatRemapBuffSize][6];//预存视频帧loadframe 处理 视频帧matremap
//int Mbuff1;//预存队首loadframe
//int Mbuff2;//队尾loadframe
//AVAudioFifo* audiofifo;//

int Mbuff11 ;//预存队首matremap
int Mbuff22 ;//队尾matremap
int xx;//sdl线程传入参数
int Sleepms = 100;//延迟
Uint32  audio_len;//每音频帧的长度
Uint8  *audio_pos;//目前接收播放位置

DWORD start_time, end_time;//帧率计时
AVFormatContext *pFormatCtx;//音频
int             i, audioStream, ct = 0;//帧率计数
AVCodecContext  *pCodecCtx;
AVCodec         *pCodec;
AVPacket        *packet;//数据包
AVFrame         *pFrame;//数据帧 音频
SDL_AudioSpec wanted_spec;
uint32_t len = 0;
int64_t in_channel_layout;
struct SwrContext *au_convert_ctx;//音频参数转换
FILE *pFile = NULL;//音频
double audio_clock;//获取音频当前时间
cv::cuda::GpuMat src[2], wM1[6], wM2[6], srcP, dst_mat[6];
cv::cuda::Stream src_stream[2], src_streamP, remap_stream[6];
SDL_Thread *thread_audio, *thread_loadframe, *thread_remap;//音频 加载视频帧 处理视频帧 三线程
ogl::Buffer wBuf[7];
int  R, ret;
float srcheight= PIC_HEIGHT, srcwidth= PIC_WIDTH;//原始图片大小
int wid = 512;//切割后图片大小
char cur_state = 0;
double rate;//原始播放帧率
bool VideoAudioStart = 0;
LARGE_INTEGER nFreq, nBeginTime, nEndTime;
GLFrame             viewFrame;
GLFrustum           viewFrustum;
GLTriangleBatch     sphereBatch;
GLBatch             cubeBatch;
GLMatrixStack       modelViewMatrix, projectionMatrix;
GLGeometryTransform transformPipeline;
GLuint              cubeTexture, reflectionShader, skyBoxShader, locMVPReflect, locMVReflect, locNormalReflect, locInvertedCamera, locMVPSkyBox;
char *szCubeFaces[6] = { "pos_x.tga", "neg_x.tga", "pos_y.tga", "neg_y.tga", "pos_z.tga", "neg_z.tga" };
enum CubemapFace { CUBEMAP_FACE_BACK, CUBEMAP_FACE_LEFT, CUBEMAP_FACE_FRONT, CUBEMAP_FACE_RIGHT, CUBEMAP_FACE_TOP, CUBEMAP_FACE_BOTTOM };
GLenum  cube[6] = { GL_TEXTURE_CUBE_MAP_POSITIVE_X,GL_TEXTURE_CUBE_MAP_NEGATIVE_X,GL_TEXTURE_CUBE_MAP_POSITIVE_Y,GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,GL_TEXTURE_CUBE_MAP_POSITIVE_Z,GL_TEXTURE_CUBE_MAP_NEGATIVE_Z };
/*Windows Socket的定义*/
WSAData wsaData;
SOCKET sockfd;
AVCodecContext *c = NULL;
AVFrame *frame = NULL;
AVPacket *pkt = NULL;
uint8_t *pic_inbuff = NULL, *pic_outbuf = NULL;
AVCodecParserContext *parser = NULL;
int   data_size;
//
cv::Mat avframe_to_cvmat(AVFrame *frame)
{
	AVFrame dst;
	cv::Mat m;

	memset(&dst, 0, sizeof(dst));

	int w = frame->width, h = frame->height;
	m = cv::Mat(h, w, CV_8UC3);
	dst.data[0] = (uint8_t *)m.data;
	avpicture_fill((AVPicture *)&dst, dst.data[0], AV_PIX_FMT_BGR24, w, h);

	struct SwsContext *convert_ctx = NULL;
	enum AVPixelFormat src_pixfmt = (enum AVPixelFormat)frame->format;
	enum AVPixelFormat dst_pixfmt = AV_PIX_FMT_BGR24;
	convert_ctx = sws_getContext(w, h, src_pixfmt, w, h, dst_pixfmt,
		SWS_FAST_BILINEAR, NULL, NULL, NULL);
	sws_scale(convert_ctx, frame->data, frame->linesize, 0, h,
		dst.data, dst.linesize);
	sws_freeContext(convert_ctx);

	return m;
}
//解码器初始化
int x264_decoder_Init(AVCodecParserContext **parser, AVCodecContext **c, AVFrame **frame, AVPacket **pkt, uint8_t **inbuf)
{

	const AVCodec *codec;
	//申请读入数据的buffer，计算buffer_size(假设buffer_size=4096)
	*inbuf = (uint8_t*)malloc(BUFFER_SIZE * sizeof(uint8_t));

	//查找x264的解码器
	codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	if (!codec) {
		fprintf(stderr, "Codec not found\n");
		exit(1);
	}
	*c = avcodec_alloc_context3(codec);
	if (!c) {
		fprintf(stderr, "Could not allocate video codec context\n");
		exit(1);
	}
	//按照编码器类型初始化解码器参数
	*parser = av_parser_init(codec->id);
	if (!parser) {
		fprintf(stderr, "parser not found\n");
		exit(1);
	}

	//对msmpeg4以及mpeg4类型的编码器必须在此处给出宽度和高度，因为编码bit流里不包含该类文件
	//在avcodec_alloc_context3之后使用，初始化AVCodeContext以便使用AVCodeContext
	if (avcodec_open2(*c, codec, NULL) < 0) {
		fprintf(stderr, "Could not open codec\n");
		exit(1);
	}

	//申请编码后的数据包以及帧
	*pkt = av_packet_alloc();
	if (!pkt)
		exit(1);
	*frame = av_frame_alloc();
	if (!frame) {
		fprintf(stderr, "Could not allocate video frame\n");
		exit(1);
	}
	return 1;
}

//flush缓冲区
void x264_decoder_Flush(AVCodecContext **c, AVFrame **frame)
{
	int ret;
	ret = avcodec_send_packet(*c, NULL);
	if (ret < 0) {
		fprintf(stderr, "Error sending a packet for decoding\n");
		exit(1);
	}

	while (ret >= 0) {
		ret = avcodec_receive_frame(*c, *frame);
		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)  //解码还未完成
			return;
		else if (ret < 0) {
			fprintf(stderr, "Error during decoding\n"); //解码出错
			exit(1);
		}
#ifdef FRAME_INFO
		printf("saving frame %3d,width=%d,height=%d\nFlush", (*c)->frame_number, (*c)->width, (*c)->height);
#endif // FRAME_INFO
		fflush(stdout);

		/*cv::Mat yuvImg;
		yuvImg.create(PIC_HEIGHT * 3 / 2, PIC_WIDTH, CV_8UC1);
		memcpy(yuvImg.data, (*frame)->data[0], PIC_SIZE * sizeof(unsigned char));
		memcpy(yuvImg.data + PIC_SIZE, (*frame)->data[1], PIC_SIZE / 4 * sizeof(unsigned char));
		memcpy(yuvImg.data + PIC_SIZE * 5 / 4, (*frame)->data[2], PIC_SIZE / 4 * sizeof(unsigned char));
		cv::Mat rgbImg, tmp;
		cv::cvtColor(yuvImg, rgbImg, CV_YUV2BGR_I420);*/


		if ((Mbuff1 + 1) % MaxLoadFrameBuffSize != Mbuff2) {
			Mbuff[Mbuff1] = avframe_to_cvmat(*frame);


			//Mbuff[Mbuff1] = rgbImg;
			Mbuff1 = (Mbuff1 + 1) % MaxLoadFrameBuffSize;
		}
	}
}
//释放解码器占用内存
void x264_decoder_Destroy(AVCodecContext **c, AVCodecParserContext **parser, AVFrame **frame, AVPacket **pkt, uint8_t **pic_inbuff)
{
	av_parser_close(*parser);
	avcodec_free_context(c);
	av_frame_free(frame);
	av_packet_free(pkt);
	free(*pic_inbuff);
}

//从左至右 拆分帧parser 编码控制器c 输入包pkt 输出解码帧frame 
int x264_decodeVideo(AVCodecParserContext **parser, AVCodecContext **c, AVPacket **pkt, AVFrame *frame, uint8_t *pic_inbuff, int data_size)
{
	int ret;
	uint8_t *data;
	data = pic_inbuff;

	if (data_size <= 0) {  //如果解码数据量为不是正整数
		return -1;
	}
	//使用parser将读入的数据解析为帧
	while (data_size> 0) {
		ret = av_parser_parse2(*parser, *c, &(*pkt)->data, &(*pkt)->size, data, data_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
		if (ret < 0) {
			fprintf(stderr, "Error while parsing\n");
			exit(1);
		}
		data += ret;   //数据指针后移
		data_size -= ret; //当前使用的数据量
#ifdef FRAME_INFO
		printf_s("ret=%d,data_size=%d,pkt->size=%d\n", ret, data_size, (*pkt)->size);
#endif
		if ((*pkt)->size) { //一帧分割完成,解码一帧
			ret = avcodec_send_packet(*c, *pkt);
			if (ret < 0) {
				fprintf(stderr, "Error sending a packet for decoding\n");
				exit(1);
			}
			while (ret >= 0) {
				ret = avcodec_receive_frame(*c, frame);
				if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)  //解码还未完成
					break;
				else if (ret < 0) {
					fprintf(stderr, "Error during decoding\n"); //解码出错
					exit(1);
				}
#ifdef FRAME_INFO
				printf("saving frame %3d,width=%d,height=%dDecode\n", (*c)->frame_number, frame->width, frame->height);
#endif // FRAME_INFO
				fflush(stdout);

				/*cv::Mat yuvImg;
				yuvImg.create(PIC_HEIGHT * 3 / 2, PIC_WIDTH, CV_8UC1);
				memcpy(yuvImg.data, frame->data[0], PIC_SIZE * sizeof(unsigned char));
				memcpy(yuvImg.data + PIC_SIZE, frame->data[1], PIC_SIZE/4 * sizeof(unsigned char));
				memcpy(yuvImg.data + PIC_SIZE * 5 / 4, frame->data[2], PIC_SIZE/4 * sizeof(unsigned char));
				cv::Mat rgbImg, tmp;
				cv::cvtColor(yuvImg, rgbImg, CV_YUV2BGR_I420);*/

				if ((Mbuff1 + 1) % MaxLoadFrameBuffSize != Mbuff2) {
					Mbuff[Mbuff1] = avframe_to_cvmat(frame);					
					Mbuff1 = (Mbuff1 + 1) % MaxLoadFrameBuffSize;
				}

			}
		}
	}
	return 0;
}

//初始化socket操作
int client_transfer_Init(WSAData *wsaData, SOCKET *sockfd)
{
	SOCKADDR_IN servaddr;
	//初始化Window Socket
	if (WSAStartup(MAKEWORD(2, 2), wsaData)) {
		printf("Fail to initialize windows socket!\n");
		return -1;
	}
	//创建一个套接字
	if ((*sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
		printf("create socket error: (errno: %d)\n", WSAGetLastError());
		return 1;
	}

	/*设置套接字为非阻塞模式*/
	if (set_non_Block(*sockfd)) {
		closesocket(*sockfd);
		return -1;
	}

	/*初始化套接字*/
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT_NUMBER);
	/*client端输入地址*/
	if (inet_pton(AF_INET, SERVER_IP, &servaddr.sin_addr) == -1) {
		printf("inet_pton error for %s\n", SERVER_IP);
		return -1;
	}

	/*连接server端*/
	if (connect_non_Block(*sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
		closesocket(*sockfd);
		return -1;
	}
	////////
	printf("Start transfer!\n");
	return 0;
}

/*设置套接字为非阻塞模式*/
int set_non_Block(SOCKET socket)
{
	/*标识符非0允许非阻塞模式*/
	int ret;
	unsigned long flag = 1;
	ret = ioctlsocket(socket, FIONBIO, (u_long*)&flag);
	if (ret)
		printf("set nonblock error: (errno: %d)\n", WSAGetLastError());
	return ret;
}

/*非阻塞套接字的connect*/
int connect_non_Block(SOCKET socket, const struct sockaddr *address, int address_len)
{
	int sel;
	struct timeval tm;
	if (connect(socket, address, address_len) < 0)
	{
		fd_set wfd;
		FD_ZERO(&wfd);
		FD_SET(socket, &wfd);
		tm.tv_sec = 3;    //3秒
		tm.tv_usec = 1;    //1u秒

		sel = select(socket + 1, NULL, &wfd, NULL, NULL);
		if (sel <0)
		{
			printf("select socket error: (errno: %d)\n", WSAGetLastError());
			return -1;
		}
		else if (sel == 0) {
			printf("connect error: (errno: %d)\n", WSAGetLastError());
		}
		else {
			printf("Connet successfully!\n");
		}
	}
	return 0;
}

/*非阻塞套接字的recv*/
int recv_non_Block(SOCKET socket, char *buffer, int length, int flags)
{
	int recv_len, ret_val, sel;
	struct timeval tm;

	for (recv_len = 0; recv_len < length;)
	{
		/*置读集*/
		fd_set read_fd;
		FD_ZERO(&read_fd);
		FD_SET(socket, &read_fd);
		//等1s收不到数据就返回
		tm.tv_sec = 1;    //秒
		tm.tv_usec = 1;    //1u秒

		sel = select(socket + 1, &read_fd, NULL, NULL, &tm);  /*调用select*/
		if (sel < 0) {   //连接失败
			printf("select socket error: (errno: %d)\n", WSAGetLastError());
			return -1;
		}
		else if (sel == 0) { //超时返回接收的最小数据
			printf("Receive time out!(errno: %d) length=%d\n", WSAGetLastError(), recv_len);
			return recv_len;
		}
		else {
			if (FD_ISSET(socket, &read_fd)) { //如果真正可读
				ret_val = recv(socket, buffer + recv_len, length - recv_len, flags);
				if (ret_val < 0) {
					printf("recv error\n");
					return ret_val;
				}
				else if (ret_val == 0) {
					printf("connection closed\n");
					return ret_val;
				}
				else
					recv_len += ret_val;
			}
		}
	}
	return recv_len;
}

/*非阻塞套接字的send*/
int send_non_Block(SOCKET socket, char *buffer, int length, int flags)
{
	int send_len, ret_val, sel;
	struct timeval tm;

	for (send_len = 0; send_len < length;)
	{
		/*置写集*/
		fd_set write_fd;
		FD_ZERO(&write_fd);
		FD_SET(socket, &write_fd);
		//等1s发不出数据就返回
		tm.tv_sec = 1;    //1秒
		tm.tv_usec = 1;    //1u秒

						   /*调用select*/
		sel = select(socket + 1, NULL, &write_fd, NULL, &tm);
		if (sel < 0) {   //连接失败
			printf("select socket error: (errno: %d)\n", WSAGetLastError());
			return -1;
		}
		else if (sel == 0) {
			printf("Send time out! (errno: %d)\n", WSAGetLastError());
			return send_len;
		}
		else {
			if (FD_ISSET(socket, &write_fd)) { //如果真正可写
				ret_val = send(socket, buffer + send_len, length - send_len, flags);
				if (ret_val < 0) {
					printf("send error\n");
					return -2;
				}
				else if (ret_val == 0) {
					printf("connection closed\n");
					return 0;
				}
				else
					send_len += ret_val;
			}
		}
	}
	return send_len;
}

//销毁客户端传输socket
void client_transfer_Destroy(SOCKET *socket)
{
	closesocket(*socket);   //客户端关闭连接
	WSACleanup();
}

/************为MatInit()服务********全景图片to六张天空盒子图**********/
float faceTransform[6][2] =
{
	{ 0, 0 },//	左右
	{ M_PI,0 },//
	{ 0,M_PI / 2 },//上
	{ 0,-M_PI / 2 },//下
	{ -M_PI / 2,0 },//前后
	{ M_PI / 2,0 },//
};
template <typename T>
inline T square(const T x)
{
	return x * x;
}
template <typename T>
inline T clamp(const T& x, const T& a, const T& b)
{
	return x < a ? a : x > b ? b : x;
}
inline Vec3f cubemapIndexToVec3(const float x, const float y, const CubemapFace face)
{
	// rotate and flip the direction as a function of the face
	Vec3f dir(x, y, 0.5f);
	Vec3f dirOut = dir;
	switch (face) {
	case CUBEMAP_FACE_BACK:
		dirOut[0] = dir[0];
		dirOut[1] = dir[2];
		dirOut[2] = -dir[1];
		break;
	case CUBEMAP_FACE_LEFT:
		dirOut[0] = -dir[2];
		dirOut[1] = dir[0];
		dirOut[2] = -dir[1];
		break;
	case CUBEMAP_FACE_TOP: break; // no-op
	case CUBEMAP_FACE_BOTTOM:
		dirOut[0] = dir[0];
		dirOut[1] = -dir[1];
		dirOut[2] = -dir[2];
		break;
	case CUBEMAP_FACE_FRONT:
		dirOut[0] = -dir[0];
		dirOut[1] = -dir[2];
		dirOut[2] = -dir[1];
		break;
	case CUBEMAP_FACE_RIGHT:
		dirOut[0] = dir[2];
		dirOut[1] = -dir[0];
		dirOut[2] = -dir[1];
		break;
	}
	return dirOut;
}
void mapEquirectToCubemapCoordinate(const float x, const float y, const CubemapFace& face, const float fisheyeFovRadians, float& srcX, float& srcY)
{
	const Vec3f dir = cubemapIndexToVec3(x, y, face);
	const float r = sqrtf(square(dir[0]) + square(dir[1]));
	const float phi = acosf(dir[2] / norm(dir));
	float theta = r > 0.0f ? acosf(fabs(dir[0] / r)) : 0.0f;
	if (dir[0] > 0 && dir[1] > 0) { // Quadrant I
									// (nothing to do)
	}
	else if (dir[0] <= 0 && dir[1] > 0) { // Quadrant II
		theta = M_PI - theta;
	}
	else if (dir[0] <= 0 && dir[1] <= 0) { // Quadrant III
		theta = M_PI + theta;
	}
	else { // Quadrant IV
		theta = 2 * M_PI - theta;
	}
	const float phiPrime = clamp(phi, 0.0f, fisheyeFovRadians);
	const float thetaPrime = clamp(theta, 0.0f, float(2.0f * M_PI));
	srcX = float(srcwidth) * thetaPrime / (2.0f * M_PI);
	srcY = float(srcheight) * phiPrime / fisheyeFovRadians;
}
/************重要函数******************/
//备用
/*
QueryPerformanceCounter(&nBeginTime);
ct = 0;

if (++ct == 30) {
ct = 0;
QueryPerformanceCounter(&nEndTime);
printf("处理端测得帧率: %f\n", 30.0 / ((double)(nEndTime.QuadPart - nBeginTime.QuadPart) / (double)nFreq.QuadPart));
nBeginTime = nEndTime;

{ 0, 0 },//	左右
{ M_PI,0 },//

{ 0,M_PI / 2 },//上
{ 0,-M_PI / 2 },//下

{ -M_PI / 2,0 },//前后
{M_PI / 2,0 },//
}
*/
//重映射函数的xmap ymap新计算方法*******问题：左右镜像显示
void NewMatInit()
{
	wid = sqrtf((float(srcheight) * float(srcwidth)) /6) + 0.5;
	//wid = 2048;
	printf("子图分辨率: %d \n", wid);
	float inWidth = srcwidth;
	float inHeight = srcheight;     
	Mat mapx(wid, wid, CV_32F);
	Mat mapy(wid, wid, CV_32F);                                        
	const float an = sin(M_PI / 4);		
	const float ak = cos(M_PI / 4);                                          //  system("pause");
	float ftu ;
	float ftv ;
	// For each point in the target image, 
	// calculate the corresponding source coordinates.                      对于每个图像计算相应的源坐标
	for (int i = 0; i < 6; ++i) {
		ftu = faceTransform[i][0];
		ftv = faceTransform[i][1];
		for (int y = 0; y < wid; y++) {
			for (int x = 0; x < wid; x++) {
				// Map face pixel coordinates to [-1, 1] on plane               将坐标映射在平面上
				float nx = (float)y / (float)wid - 0.5f;
				float ny = (float)x / (float)wid - 0.5f;
				nx *= 2;
				ny *= 2;
				// Map [-1, 1] plane coord to [-an, an]                          
				// thats the coordinates in respect to a unit sphere 
				// that contains our box. 
				nx *= an;
				ny *= an;
				float u, v;
				// Project from plane to sphere surface.
				if (ftv == 0) {
					// Center faces
					u = atan2(nx, ak);
					v = atan2(ny * cos(u), ak);
					u += ftu;
				}
				else if (ftv > 0) {
					// Bottom face 
					float d = sqrt(nx * nx + ny * ny);
					v = -M_PI / 2 + atan2(d, ak);
					u = atan2(-ny, nx);
				}
				else {
					// Top face
					float d = sqrt(nx * nx + ny * ny);
					v = M_PI / 2 - atan2(d, ak);
					u = atan2(ny, nx);
					
				}
				// Map from angular coordinates to [-1, 1], respectively.
				u = u / (M_PI);
				v = v / (M_PI / 2);
				// Warp around, if our coordinates are out of bounds. 
				while (v < -1) {
					v += 2;
					u += 1;
				}
				while (v > 1) {
					v -= 2;
					u += 1;
				}
				while (u < -1) {
					u += 2;
				}
				while (u > 1) {
					u -= 2;
				}
				// Map from [-1, 1] to in texture space
				u = u / 2.0f + 0.5f;
				v = v / 2.0f + 0.5f;
				u = u*(inWidth - 1);
				v = v*(inHeight - 1);
				mapx.at<float>(x, y) = u;
				mapy.at<float>(x, y) = v;
			}
		}
		wM1[i].upload(mapx);
		wM2[i].upload(mapy);
	}
}
//音频回调函数   用户数据   要填充的音频缓存区头指针  缓存区大小
void  fill_audio(void *udata, Uint8 *stream, int len) {
	SDL_memset(stream, 0, len);//初始化缓存区
	if (audio_len == 0)
		return;
	len = (len>audio_len ? audio_len : len);
	SDL_MixAudio(stream, audio_pos, len, SDL_MIX_MAXVOLUME);
	audio_pos += len;
	audio_len -= len;
	//cout << ct << endl;
	//SDL_Delay(5);
}
//初始化 ffmpeg读取音频
int AudioInit() {
	av_register_all();
	avformat_network_init();
	pFormatCtx = avformat_alloc_context();
	if (avformat_open_input(&pFormatCtx, VideoAddr, NULL, NULL) != 0) {
		printf("Couldn't open input stream.\n");
		return -1;
	}
	if (avformat_find_stream_info(pFormatCtx, NULL)<0) {
		printf("Couldn't find stream information.\n");
		return -1;
	}
	// Find the first audio stream
	audioStream = -1;
	for (i = 0; i < pFormatCtx->nb_streams; i++)
		if (pFormatCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
			audioStream = i;
			break;
		}
	if (audioStream == -1) {
		printf("Didn't find a audio stream.\n");
		return -1;
	}
	// Get a pointer to the codec context for the audio stream
	pCodecCtx = avcodec_alloc_context3(NULL);
	avcodec_parameters_to_context(pCodecCtx, pFormatCtx->streams[audioStream]->codecpar);
	// Find the decoder for the audio stream
	pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
	if (pCodec == NULL) {
		printf("Codec not found.\n");
		return -1;
	}
	// Open codec
	if (avcodec_open2(pCodecCtx, pCodec, NULL)<0) {
		printf("Could not open codec.\n");
		return -1;
	}
	packet = (AVPacket *)av_malloc(sizeof(AVPacket));
	av_init_packet(packet);
}
//音频播放线程
int AudioPlay(void* xx) {
	int out_framesize = 1024;//nb_samples: AAC-1024 MP3-1152 一般都是aac居多
							 //音频输出格式设置out_XXX
	uint64_t out_channel_layout = AV_CH_LAYOUT_STEREO;
	int out_nb_samples = out_framesize;
	AVSampleFormat out_sample_fmt = AV_SAMPLE_FMT_S16;
	int out_sample_rate = 44100;// pCodecCtx->sample_rate;
	int out_channels = av_get_channel_layout_nb_channels(out_channel_layout);
	//Out Buffer Size
	int out_buffer_size = av_samples_get_buffer_size(NULL, out_channels, out_nb_samples, out_sample_fmt, 1);
	uint8_t ** audio_data_buffer = NULL;//存储转换后的数据，再编码AAC   二维数组data[0],data[1]
	audiofifo = av_audio_fifo_alloc(out_sample_fmt, out_channels, 1);
	pFrame = av_frame_alloc();
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER)) {
		printf("Could not initialize SDL - %s\n", SDL_GetError());
		return -1;
	}
	//SDL_AudioSpec
	wanted_spec.freq = out_sample_rate;//音频采样率
	wanted_spec.format = AUDIO_S16SYS;//采样精度
	wanted_spec.channels = out_channels;//音频频道数
	wanted_spec.silence = 0;//不静音
	wanted_spec.samples = out_nb_samples;//AAC 1024
	wanted_spec.callback = fill_audio;//音频回调函数 音频播放内部自动开线程循环调用
	wanted_spec.userdata = pCodecCtx;//用户数据
	if (SDL_OpenAudio(&wanted_spec, NULL) < 0) {
		printf("can't open audio.\n");
		return -1;
	}
	//FIX:Some Codec's Context Information is missing
	in_channel_layout = av_get_default_channel_layout(pCodecCtx->channels);
	/*SwrContext   主要用于音频重采样，比如采样率转换； SwsContext 主要用于视频图像的转换，比如格式转换 https://blog.csdn.net/tea1896/article/details/72453510
	*/
	au_convert_ctx = swr_alloc();
	au_convert_ctx = swr_alloc_set_opts(au_convert_ctx,
		out_channel_layout,//输出
		out_sample_fmt, //编码前你希望的格式
		out_sample_rate,//输出
		in_channel_layout, //输入
		pCodecCtx->sample_fmt,//PCM源文件的采样格式 
		pCodecCtx->sample_rate, //输入
		0, NULL);
	swr_init(au_convert_ctx);
	SDL_PauseAudio(0);//开始播放音频 应该是和下面的循环读取编解码并行线程
	AVFrame* audio_frame = av_frame_alloc();
	//循环读取音频帧并且编解码（发送包接收帧）
	while (av_read_frame(pFormatCtx, packet) >= 0) {
		if (packet->stream_index == audioStream) {
			int ret = avcodec_send_packet(pCodecCtx, packet);//发送 包
			if (ret != 0) { printf("%s\n", "error"); }
			while (!VideoAudioStart);
			while (avcodec_receive_frame(pCodecCtx, pFrame) == 0) {//接收 帧
				av_samples_alloc_array_and_samples(&audio_data_buffer,/* 此时data[0],data[1]分别指向audio_data_buffer数组起始、中间地址 */
					NULL, out_channels,
					pFrame->nb_samples,
					out_sample_fmt, 1);
				int convert_size = swr_convert(au_convert_ctx,/* 转换数据，令各自声道的音频数据存储在不同的数组（分别由不同指针指向）*/
					audio_data_buffer,
					pFrame->nb_samples,
					(const uint8_t**)pFrame->data,
					pFrame->nb_samples);
				ret = av_audio_fifo_realloc(audiofifo, av_audio_fifo_size(audiofifo) + convert_size);
				if (ret < 0) {
					printf("av_audio_fifo_realloc error\n");
					return -1;
				}
				if (av_audio_fifo_write(audiofifo, (void **)audio_data_buffer, convert_size) < convert_size) {
					printf("av_audio_fifo_write error\n");
					return -1;
				}
				while (av_audio_fifo_size(audiofifo) >= out_framesize) {
					int frame_size = FFMIN(av_audio_fifo_size(audiofifo), out_framesize);
					audio_frame->nb_samples = frame_size;
					audio_frame->channel_layout = out_channel_layout;
					audio_frame->format = out_sample_fmt;
					audio_frame->sample_rate = out_sample_rate;
					av_frame_get_buffer(audio_frame, 0);
					if (av_audio_fifo_read(audiofifo, (void **)audio_frame->data, frame_size) < frame_size) {
						printf("av_audio_fifo_read error\n");
						return -1;
					}
					if (wanted_spec.samples != frame_size) {
						SDL_CloseAudio();
						out_nb_samples = frame_size;
						out_buffer_size = av_samples_get_buffer_size(NULL, out_channels, out_nb_samples, out_sample_fmt, 1);
						wanted_spec.samples = out_nb_samples;
						SDL_OpenAudio(&wanted_spec, NULL);
					}
					while (audio_len>0)//Wait until finish
						SDL_Delay(1);
					audio_len = out_buffer_size;
					audio_pos = *audio_frame->data;
					//SDL_Delay(40);
				}
			}
		}
		av_packet_unref(packet);
	}
}
//音频释放
void AudioEnd() {
	swr_free(&au_convert_ctx);
	SDL_CloseAudio();//Close SDL
	SDL_Quit();
	if (audiofifo)
		av_audio_fifo_free(audiofifo);
	avcodec_close(pCodecCtx);
	avformat_close_input(&pFormatCtx);
}
//初始化处理视频帧
int MatInit()  //不要动
{
	CubemapFace face;
	//wid = sqrtf((float(srcheight) * float(srcwidth)) / 6) + 0.5;
	//wid = 1024;
	printf("子图分辨率: %d \n", wid);
	const float dy = 1.0f / float(wid);
	const float dx = 1.0f / float(wid);
	for (int ii = 0; ii < 6; ii++)
	{
		warpMat1[ii] = Mat(Size(wid, wid), CV_32FC1);
		warpMat2[ii] = Mat(Size(wid, wid), CV_32FC1);
		//warpMat[ii] = Mat(Size(wid, wid), CV_32FC2);
		//wBuf[ii].create(Size(wid, wid), picture_remap.rgbImg.type(), ogl::Buffer::Target::PIXEL_UNPACK_BUFFER);
		switch (ii)
		{
		case 0:
			face = CUBEMAP_FACE_RIGHT;
			break;
		case 1:
			face = CUBEMAP_FACE_LEFT;
			break;
		case 2:
			face = CUBEMAP_FACE_TOP;
			break;
		case 3:
			face = CUBEMAP_FACE_BOTTOM;
			break;
		case 4:
			face = CUBEMAP_FACE_BACK;
			break;
		case 5:
			face = CUBEMAP_FACE_FRONT;
			break;
		}
		for (int j = 0; j < wid; ++j)
		{
			for (int i = 0; i < wid; ++i)
			{
				float srcX;
				float srcY;
				mapEquirectToCubemapCoordinate(	float(i) * dy - 0.5f,	float(j) * dx - 0.5f,	face,M_PI,	srcX, srcY);
				warpMat1[ii].at<float>(j, i) = static_cast<float>(srcX);
				warpMat2[ii].at<float>(j, i) = static_cast<float>(srcY);
				//warpMat[ii].at<Point2f>(j, i) = Point2f(srcX, srcY);
			}
		}
		wM1[ii].upload(warpMat1[ii]);
		wM2[ii].upload(warpMat2[ii]);
	}
	return 1;
}
//加载视频帧线程
int LoadFrame(void*xx) {
	x264_decoder_Init(&parser, &c, &frame, &pkt, &pic_inbuff);
	if (client_transfer_Init(&wsaData, &sockfd)==-1) {
	printf("Socket error!\n");
	exit(1);
	}
	data_size = 0;
	while (1) {
		//读入编码后的二进制码流
		data_size = recv_non_Block(sockfd,(char*)pic_inbuff,BUFFER_SIZE,0);
		if (data_size < 0) {   //客户端关闭连接
			client_transfer_Destroy(&sockfd);
			break;
		}
		else {//使用parser将读入的数据解析为帧
			x264_decodeVideo(&parser, &c, &pkt, frame, pic_inbuff,data_size);
		}
	}
		x264_decoder_Flush(&c, &frame);
		x264_decoder_Destroy(&c, &parser, &frame, &pkt, &pic_inbuff);
		delete pic_outbuf;
		return 0;
}
//处理视频帧线程
int MatRemap(void*xx) {
	
	while (1) {
		while (Mbuff2 != Mbuff1 && (Mbuff11 + 1) % MaxMatRemapBuffSize != Mbuff22) {
			M = Mbuff[Mbuff2];
			srcP.upload(M, src_streamP);
			//src_streamP.waitForCompletion();
			for (int ii = 0; ii < 6; ii++) {
				cv::cuda::remap(srcP, dst_mat[ii], wM1[ii], wM2[ii], CV_INTER_CUBIC, BORDER_WRAP, cv::Scalar(), remap_stream[ii]);
				dst_mat[ii].download(m[Mbuff11][ii]);
			}
			Mbuff2 = (Mbuff2 + 1) % MaxLoadFrameBuffSize;
			Mbuff11 = (Mbuff11 + 1) % MaxMatRemapBuffSize;			
		}		
	}
}
//出队列 视频帧
void PopRemap() {
	while (Mbuff22 == Mbuff11)
		SDL_Delay(1);
	for (int ii = 0; ii < 6; ii++) {
		//dst_mat[ii].download(m[Mbuff1][ii]);
		glTexImage2D(cube[ii], 0, GL_RGB, wid, wid, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m[Mbuff22][ii].data);
		//glTexSubImage2D(cube[ii], 0,0, 0, R, R, GL_BGR_EXT, GL_UNSIGNED_BYTE, m[Mbuff22][ii].data);
	}
	//glTexImage2D(cube[3], 0, GL_RGB, m[Mbuff22][3].rows, m[Mbuff22][3].rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, 0);
	//for (int ii = 4; ii < 6; ii++) {		
		//glTexImage2D(cube[ii], 0, GL_RGB, m[Mbuff22][ii].rows, m[Mbuff22][ii].rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, m[Mbuff22][ii].data);
	//}
	Mbuff22 = (Mbuff22 + 1) % MaxMatRemapBuffSize;
	glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
}
//总初始化
void SetupRC() {
	glGenTextures(1, &cubeTexture);
	glBindTexture(GL_TEXTURE_CUBE_MAP, cubeTexture);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	//
	gltMakeCube(cubeBatch, 16.0f);
	skyBoxShader = gltLoadShaderPairWithAttributes("SkyBox.vp", "SkyBox.fp", 2, GLT_ATTRIBUTE_VERTEX, "vVertex", GLT_ATTRIBUTE_NORMAL, "vNormal");
	locMVPSkyBox = glGetUniformLocation(skyBoxShader, "mvpMatrix");
}
/************次要函数******************/
//循环显示视频
void CMainApplication::RunMainLoop()
{
	bool bQuit = false;
	SDL_StartTextInput();
	SDL_ShowCursor(SDL_DISABLE);
	long int start_time, end_time, diff;
	int rate_real = rate;
	int rate_ms = int(1000 / rate);
	int ct1 = 0;
	start_time = GetTickCount();
	//QueryPerformanceCounter(&nBeginTime);
	//QueryPerformanceCounter(&nEndTime);
	
	//printf("upload time: %f\n", (double)(nEndTime.QuadPart - nBeginTime.QuadPart) / (double)nFreq.QuadPart);
	for (VideoAudioStart = 1; !bQuit;) {

		PopRemap();
		RenderFrame();
		RenderFrame();
		RenderFrame();
		RenderFrame();
		RenderFrame();
		//printf("Mf1 %d \n", Mbuff1);
		//printf("Mf2 %d \n", Mbuff2);
		//printf("Mf11 %d \n", Mbuff11);
		//printf("Mf22 %d \n", Mbuff22);
		//if (++ct1 == 30) {
		//	ct1 = 0;
		//	end_time = GetTickCount();
		//	printf("播放端测得帧率 %f \n", 30000.0 / (end_time - start_time));
		//	start_time = end_time;
		//	//QueryPerformanceCounter(&nEndTime);
		//	//printf("测得帧率: %f\n", 30.0/((double)(nEndTime.QuadPart - nBeginTime.QuadPart) / (double)nFreq.QuadPart));
		//	//nBeginTime = nEndTime;
		//}
		
	}
	SDL_StopTextInput();
}
bool CMainApplication::HandleInput()
{
	SDL_Event sdlEvent;
	bool bRet = false;
	while (SDL_PollEvent(&sdlEvent) != 0)
	{
		if (sdlEvent.type == SDL_QUIT)
		{
			bRet = true;
		}
		else if (sdlEvent.type == SDL_KEYDOWN)
		{
			if (sdlEvent.key.keysym.sym == SDLK_ESCAPE
				|| sdlEvent.key.keysym.sym == SDLK_q)
			{
				bRet = true;
			}
			if (sdlEvent.key.keysym.sym == SDLK_c)
			{
				m_bShowCubes = !m_bShowCubes;
			}
			if (sdlEvent.key.keysym.sym == SDLK_SPACE) {//   播放/暂停
				;
			}
		}
	}
	// Process SteamVR events
	vr::VREvent_t event;
	while (m_pHMD->PollNextEvent(&event, sizeof(event)))
	{
		ProcessVREvent(event);
	}
	// Process SteamVR controller state
	for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
	{
		vr::VRControllerState_t state;
		if (m_pHMD->GetControllerState(unDevice, &state, sizeof(state)))
		{
			m_rbShowTrackedDevice[unDevice] = state.ulButtonPressed == 0;
		}
	}
	return bRet;
}
void CMainApplication::RenderFrame()
{
	// for now as fast as possible
	long int start_time1, end_time1;
	
	if (m_pHMD)
	{
		//修改点
		//RenderControllerAxes();
		RenderStereoTargets();
		RenderCompanionWindow();
		
		//贴图上传到眼镜
		vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)leftEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)rightEyeDesc.m_nResolveTextureId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
		vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
		vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);
		
	}
	if (m_bVblank && m_bGlFinishHack)
	{
		glFinish();
	}
	{
		SDL_GL_SwapWindow(m_pCompanionWindow);
	}
	// Clear 修改点
	/*{
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}*/

	// Flush and wait for swap.
	if (m_bVblank)
	{
		glFlush();
		glFinish();
	}
	// Spew out the controller and pose count whenever they change.
	/*
	if (m_iTrackedControllerCount != m_iTrackedControllerCount_Last || m_iValidPoseCount != m_iValidPoseCount_Last)
	{
	m_iValidPoseCount_Last = m_iValidPoseCount;
	m_iTrackedControllerCount_Last = m_iTrackedControllerCount;
	dprintf("PoseCount:%d(%s) Controllers:%d\n", m_iValidPoseCount, m_strPoseClasses.c_str(), m_iTrackedControllerCount);
	}*/
	//start_time1 = GetTickCount();
	UpdateHMDMatrixPose();
	//end_time1 = GetTickCount();
	//printf("time1 %d \n", end_time1);
	//printf("time1 %d \n", end_time1-start_time1);
}
void CMainApplication::SetupScene()
{
	if (!m_pHMD)
		return;
	SetupRC();
	cubeBatch.Draw();//显示屏显示
					 //modelViewMatrix.PopMatrix();
					 /*
					 std::vector<float> vertdataarray;
					 Matrix4 matScale;
					 matScale.scale(m_fScale, m_fScale, m_fScale);
					 Matrix4 matTransform;
					 matTransform.translate(
					 -((float)m_iSceneVolumeWidth * m_fScaleSpacing) / 2.f,
					 -((float)m_iSceneVolumeHeight * m_fScaleSpacing) / 2.f,
					 -((float)m_iSceneVolumeDepth * m_fScaleSpacing) / 2.f);
					 Matrix4 mat = matScale * matTransform;
					 for (int z = 0; z< m_iSceneVolumeDepth; z++)
					 {
					 for (int y = 0; y< m_iSceneVolumeHeight; y++)
					 {
					 for (int x = 0; x< m_iSceneVolumeWidth; x++)
					 {
					 AddCubeToScene(mat, vertdataarray);
					 mat = mat * Matrix4().translate(m_fScaleSpacing, 0, 0);
					 }
					 mat = mat * Matrix4().translate(-((float)m_iSceneVolumeWidth) * m_fScaleSpacing, m_fScaleSpacing, 0);
					 }
					 mat = mat * Matrix4().translate(0, -((float)m_iSceneVolumeHeight) * m_fScaleSpacing, m_fScaleSpacing);
					 }
					 m_uiVertcount = vertdataarray.size() / 5;
					 glGenVertexArrays(1, &m_unSceneVAO);
					 glBindVertexArray(m_unSceneVAO);
					 glGenBuffers(1, &m_glSceneVertBuffer);
					 glBindBuffer(GL_ARRAY_BUFFER, m_glSceneVertBuffer);
					 glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STATIC_DRAW);
					 GLsizei stride = sizeof(VertexDataScene);
					 uintptr_t offset = 0;
					 glEnableVertexAttribArray(0);
					 glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);
					 offset += sizeof(Vector3);
					 glEnableVertexAttribArray(1);
					 glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, (const void *)offset);
					 glBindVertexArray(0);
					 glDisableVertexAttribArray(0);
					 glDisableVertexAttribArray(1);
					 */
}
void CMainApplication::RenderControllerAxes()
{
	// don't draw controllers if somebody else has input focus
	if (m_pHMD->IsInputFocusCapturedByAnotherProcess())
		return;
	std::vector<float> vertdataarray;
	m_uiControllerVertcount = 0;
	m_iTrackedControllerCount = 0;
	for (vr::TrackedDeviceIndex_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; ++unTrackedDevice)
	{
		if (!m_pHMD->IsTrackedDeviceConnected(unTrackedDevice))
			continue;
		if (m_pHMD->GetTrackedDeviceClass(unTrackedDevice) != vr::TrackedDeviceClass_Controller)
			continue;
		m_iTrackedControllerCount += 1;
		if (!m_rTrackedDevicePose[unTrackedDevice].bPoseIsValid)
			continue;
		const Matrix4 & mat = m_rmat4DevicePose[unTrackedDevice];
		Vector4 center = mat * Vector4(0, 0, 0, 1);
		for (int i = 0; i < 3; ++i)
		{
			Vector3 color(0, 0, 0);
			Vector4 point(0, 0, 0, 1);
			point[i] += 0.05f;  // offset in X, Y, Z
			color[i] = 1.0;  // R, G, B
			point = mat * point;
			vertdataarray.push_back(center.x);
			vertdataarray.push_back(center.y);
			vertdataarray.push_back(center.z);
			vertdataarray.push_back(color.x);
			vertdataarray.push_back(color.y);
			vertdataarray.push_back(color.z);
			vertdataarray.push_back(point.x);
			vertdataarray.push_back(point.y);
			vertdataarray.push_back(point.z);
			vertdataarray.push_back(color.x);
			vertdataarray.push_back(color.y);
			vertdataarray.push_back(color.z);
			m_uiControllerVertcount += 2;
		}
		Vector4 start = mat * Vector4(0, 0, -0.02f, 1);
		Vector4 end = mat * Vector4(0, 0, -39.f, 1);
		Vector3 color(.92f, .92f, .71f);
		vertdataarray.push_back(start.x); vertdataarray.push_back(start.y); vertdataarray.push_back(start.z);
		vertdataarray.push_back(color.x); vertdataarray.push_back(color.y); vertdataarray.push_back(color.z);
		vertdataarray.push_back(end.x); vertdataarray.push_back(end.y); vertdataarray.push_back(end.z);
		vertdataarray.push_back(color.x); vertdataarray.push_back(color.y); vertdataarray.push_back(color.z);
		m_uiControllerVertcount += 2;
	}
	// Setup the VAO the first time through.
	if (m_unControllerVAO == 0)
	{
		glGenVertexArrays(1, &m_unControllerVAO);
		glBindVertexArray(m_unControllerVAO);
		glGenBuffers(1, &m_glControllerVertBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_glControllerVertBuffer);
		GLuint stride = 2 * 3 * sizeof(float);
		uintptr_t offset = 0;
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);
		offset += sizeof(Vector3);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (const void *)offset);
		glBindVertexArray(0);
	}
	glBindBuffer(GL_ARRAY_BUFFER, m_glControllerVertBuffer);
	// set vertex data if we have some
	if (vertdataarray.size() > 0)
	{
		//$ TODO: Use glBufferSubData for this...
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertdataarray.size(), &vertdataarray[0], GL_STREAM_DRAW);
	}
}
void CMainApplication::RenderStereoTargets()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_MULTISAMPLE);
	// Left Eye
	glBindFramebuffer(GL_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId);
	glViewport(0, 0, m_nRenderWidth, m_nRenderHeight);
	RenderScene(vr::Eye_Left);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDisable(GL_MULTISAMPLE);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, leftEyeDesc.m_nRenderFramebufferId);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, leftEyeDesc.m_nResolveFramebufferId);
	glBlitFramebuffer(0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight,
		GL_COLOR_BUFFER_BIT,
		GL_LINEAR);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
	glEnable(GL_MULTISAMPLE);
	// Right Eye
	glBindFramebuffer(GL_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId);
	glViewport(0, 0, m_nRenderWidth, m_nRenderHeight);
	RenderScene(vr::Eye_Right);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDisable(GL_MULTISAMPLE);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, rightEyeDesc.m_nRenderFramebufferId);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, rightEyeDesc.m_nResolveFramebufferId);
	glBlitFramebuffer(0, 0, m_nRenderWidth, m_nRenderHeight, 0, 0, m_nRenderWidth, m_nRenderHeight,
		GL_COLOR_BUFFER_BIT,
		GL_LINEAR);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}
void CMainApplication::RenderScene(vr::Hmd_Eye nEye)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	if (m_bShowCubes)
	{
		//modelViewMatrix.PushMatrix();
		glUseProgram(skyBoxShader);
		glUniformMatrix4fv(locMVPSkyBox, 1, GL_FALSE, GetCurrentViewProjectionMatrix(nEye).get());
		cubeBatch.Draw();
		//modelViewMatrix.PopMatrix();
		/*
		glUseProgram(m_unSceneProgramID);
		glUniformMatrix4fv(m_nSceneMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix(nEye).get());
		glBindVertexArray(m_unSceneVAO);
		glBindTexture(GL_TEXTURE_2D, m_iTexture);
		glDrawArrays(GL_TRIANGLES, 0, m_uiVertcount);
		glBindVertexArray(0);*/
	}
	bool bIsInputCapturedByAnotherProcess = m_pHMD->IsInputFocusCapturedByAnotherProcess();
	if (!bIsInputCapturedByAnotherProcess)
	{
		// draw the controller axis lines
		glUseProgram(m_unControllerTransformProgramID);
		glUniformMatrix4fv(m_nControllerMatrixLocation, 1, GL_FALSE, GetCurrentViewProjectionMatrix(nEye).get());
		glBindVertexArray(m_unControllerVAO);
		glDrawArrays(GL_LINES, 0, m_uiControllerVertcount);
		glBindVertexArray(0);
	}
	// ----- Render Model rendering -----
	glUseProgram(m_unRenderModelProgramID);
	for (uint32_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
	{
		if (!m_rTrackedDeviceToRenderModel[unTrackedDevice] || !m_rbShowTrackedDevice[unTrackedDevice])
			continue;
		const vr::TrackedDevicePose_t & pose = m_rTrackedDevicePose[unTrackedDevice];
		if (!pose.bPoseIsValid)
			continue;
		if (bIsInputCapturedByAnotherProcess && m_pHMD->GetTrackedDeviceClass(unTrackedDevice) == vr::TrackedDeviceClass_Controller)
			continue;
		const Matrix4 & matDeviceToTracking = m_rmat4DevicePose[unTrackedDevice];
		Matrix4 matMVP = GetCurrentViewProjectionMatrix(nEye) * matDeviceToTracking;
		glUniformMatrix4fv(m_nRenderModelMatrixLocation, 1, GL_FALSE, matMVP.get());
		m_rTrackedDeviceToRenderModel[unTrackedDevice]->Draw();//VR眼镜显示
	}
	glUseProgram(0);
}
void CMainApplication::RenderCompanionWindow()//屏幕显示屏左右眼
{
	glDisable(GL_DEPTH_TEST);
	glViewport(0, 0, m_nCompanionWindowWidth, m_nCompanionWindowHeight);
	glBindVertexArray(m_unCompanionWindowVAO);
	glUseProgram(m_unCompanionWindowProgramID);
	// render left eye (first half of index array )
	glBindTexture(GL_TEXTURE_2D, leftEyeDesc.m_nResolveTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glDrawElements(GL_TRIANGLES, m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, (const void *)(uintptr_t)(m_uiCompanionWindowIndexSize));
	glDrawElements(GL_TRIANGLES, m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, 0);
	// render right eye (second half of index array )
	glBindTexture(GL_TEXTURE_2D, rightEyeDesc.m_nResolveTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glDrawElements(GL_TRIANGLES, m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, 0);
	glDrawElements(GL_TRIANGLES, m_uiCompanionWindowIndexSize / 2, GL_UNSIGNED_SHORT, (const void *)(uintptr_t)(m_uiCompanionWindowIndexSize));
	glBindVertexArray(0);
	glUseProgram(0);
}
void CGLRenderModel::Draw()
{
	glBindVertexArray(m_glVertArray);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_glTexture);//
	glDrawElements(GL_TRIANGLES, m_unVertexCount, GL_UNSIGNED_SHORT, 0);
	glBindVertexArray(0);
}
void CMainApplication::UpdateHMDMatrixPose()
{
	if (!m_pHMD)
		return;
	vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
	m_iValidPoseCount = 0;
	m_strPoseClasses = "";
	for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
	{
		if (m_rTrackedDevicePose[nDevice].bPoseIsValid)
		{
			m_iValidPoseCount++;
			m_rmat4DevicePose[nDevice] = ConvertSteamVRMatrixToMatrix4(m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
			if (m_rDevClassChar[nDevice] == 0)
			{
				//修改点
				/*
				switch (m_pHMD->GetTrackedDeviceClass(nDevice))
				{
				case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
				case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
				case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
				case vr::TrackedDeviceClass_GenericTracker:    m_rDevClassChar[nDevice] = 'G'; break;
				case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
				default:                                       m_rDevClassChar[nDevice] = '?'; break;
				}*/
			}
			m_strPoseClasses += m_rDevClassChar[nDevice];
		}
	}
	if (m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
	{
		m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
		m_mat4HMDPose.invert();
	}
}
/************其他函数******************/
void dprintf(const char *fmt, ...)
{
	va_list args;
	char buffer[2048];
	va_start(args, fmt);
	vsprintf_s(buffer, fmt, args);
	va_end(args);
	if (g_bPrintf)
		printf("%s", buffer);
	OutputDebugStringA(buffer);
}
CMainApplication::CMainApplication(int argc, char *argv[])
	: m_pCompanionWindow(NULL)
	, m_pContext(NULL)
	, m_nCompanionWindowWidth(1200)//显示屏窗口大小
	, m_nCompanionWindowHeight(600)
	, m_unSceneProgramID(0)
	, m_unCompanionWindowProgramID(0)
	, m_unControllerTransformProgramID(0)
	, m_unRenderModelProgramID(0)
	, m_pHMD(NULL)
	, m_pRenderModels(NULL)
	, m_bDebugOpenGL(false)
	, m_bVerbose(false)
	, m_bPerf(false)
	, m_bVblank(false)
	, m_bGlFinishHack(true)
	, m_glControllerVertBuffer(0)
	, m_unControllerVAO(0)
	, m_unSceneVAO(0)
	, m_nSceneMatrixLocation(-1)
	, m_nControllerMatrixLocation(-1)
	, m_nRenderModelMatrixLocation(-1)
	, m_iTrackedControllerCount(0)
	, m_iTrackedControllerCount_Last(-1)
	, m_iValidPoseCount(0)
	, m_iValidPoseCount_Last(-1)
	, m_iSceneVolumeInit(2)//
	, m_strPoseClasses("")
	, m_bShowCubes(true)
{
	for (int i = 1; i < argc; i++)
	{
		if (!stricmp(argv[i], "-gldebug"))
		{
			m_bDebugOpenGL = true;
		}
		else if (!stricmp(argv[i], "-verbose"))
		{
			m_bVerbose = true;
		}
		else if (!stricmp(argv[i], "-novblank"))
		{
			m_bVblank = false;
		}
		else if (!stricmp(argv[i], "-noglfinishhack"))
		{
			m_bGlFinishHack = false;
		}
		else if (!stricmp(argv[i], "-noprintf"))
		{
			g_bPrintf = false;
		}
		else if (!stricmp(argv[i], "-cubevolume") && (argc > i + 1) && (*argv[i + 1] != '-'))
		{
			m_iSceneVolumeInit = atoi(argv[i + 1]);
			i++;
		}
	}
	// other initialization tasks are done in BInit
	memset(m_rDevClassChar, 0, sizeof(m_rDevClassChar));
};
CMainApplication::~CMainApplication()
{
	// work is done in Shutdown
	dprintf("Shutdown");
}
std::string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
{
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
	if (unRequiredBufferLen == 0)
		return "";
	char *pchBuffer = new char[unRequiredBufferLen];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
	std::string sResult = pchBuffer;
	delete[] pchBuffer;
	return sResult;
}
bool CMainApplication::BInit()
{
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
	{
		printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}
	// Loading the SteamVR Runtime
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);
	if (eError != vr::VRInitError_None)
	{
		m_pHMD = NULL;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
		return false;
	}
	m_pRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);
	if (!m_pRenderModels)
	{
		m_pHMD = NULL;
		vr::VR_Shutdown();
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
		return false;
	}
	int nWindowPosX = 700;//显示屏的坐标
	int nWindowPosY = 100;
	Uint32 unWindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
	//SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY );
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
	if (m_bDebugOpenGL)
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
	m_pCompanionWindow = SDL_CreateWindow("hellovr - CIC474", nWindowPosX, nWindowPosY, m_nCompanionWindowWidth, m_nCompanionWindowHeight, unWindowFlags);//
	if (m_pCompanionWindow == NULL)
	{
		printf("%s - Window could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}
	m_pContext = SDL_GL_CreateContext(m_pCompanionWindow);
	if (m_pContext == NULL)
	{
		printf("%s - OpenGL context could not be created! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}
	glewExperimental = GL_TRUE;
	GLenum nGlewError = glewInit();
	if (nGlewError != GLEW_OK)
	{
		printf("%s - Error initializing GLEW! %s\n", __FUNCTION__, glewGetErrorString(nGlewError));
		return false;
	}
	glGetError(); // to clear the error caused deep in GLEW
	if (SDL_GL_SetSwapInterval(m_bVblank ? 1 : 0) < 0)
	{
		printf("%s - Warning: Unable to set VSync! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}
	//m_strDriver = "No Driver";
	//m_strDisplay = "No Display";
	//m_strDriver = GetTrackedDeviceString(m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
	//m_strDisplay = GetTrackedDeviceString(m_pHMD, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
	//std::string strWindowTitle = "hellovr - " + m_strDriver + " " + m_strDisplay;
	std::string strWindowTitle = "HELLOVR - CIC474";
	SDL_SetWindowTitle(m_pCompanionWindow, strWindowTitle.c_str());
	// cube array
	m_iSceneVolumeWidth = m_iSceneVolumeInit;
	m_iSceneVolumeHeight = m_iSceneVolumeInit;
	m_iSceneVolumeDepth = m_iSceneVolumeInit;
	m_fScale = 0.3f;
	m_fScaleSpacing = 4.0f;
	m_fNearClip = 0.1f;
	m_fFarClip = 30.0f;
	m_iTexture = 0;
	m_uiVertcount = 0;
	// 		m_MillisecondsTimer.start(1, this);
	// 		m_SecondsTimer.start(1000, this);
	if (!BInitGL())
	{
		printf("%s - Unable to initialize OpenGL!\n", __FUNCTION__);
		return false;
	}
	if (!BInitCompositor())
	{
		printf("%s - Failed to initialize VR Compositor!\n", __FUNCTION__);
		return false;
	}
	return true;
}
void APIENTRY DebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const char* message, const void* userParam)
{
	dprintf("GL Error: %s\n", message);
}
bool CMainApplication::BInitGL()
{
	if (m_bDebugOpenGL)
	{
		glDebugMessageCallback((GLDEBUGPROC)DebugCallback, nullptr);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	}
	if (!CreateAllShaders())
		return false;
	//SetupTexturemaps();
	SetupScene();
	SetupCameras();
	SetupStereoRenderTargets();
	SetupCompanionWindow();
	SetupRenderModels();
	return true;
}
bool CMainApplication::BInitCompositor()
{
	vr::EVRInitError peError = vr::VRInitError_None;
	if (!vr::VRCompositor())
	{
		printf("Compositor initialization failed. See log file for details\n");
		return false;
	}
	return true;
}
void CMainApplication::Shutdown()
{
	if (m_pHMD)
	{
		vr::VR_Shutdown();
		m_pHMD = NULL;
	}
	for (std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++)
	{
		delete (*i);
	}
	m_vecRenderModels.clear();
	if (m_pContext)
	{
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_FALSE);
		glDebugMessageCallback(nullptr, nullptr);
		glDeleteBuffers(1, &m_glSceneVertBuffer);
		if (m_unSceneProgramID)
		{
			glDeleteProgram(m_unSceneProgramID);
		}
		if (m_unControllerTransformProgramID)
		{
			glDeleteProgram(m_unControllerTransformProgramID);
		}
		if (m_unRenderModelProgramID)
		{
			glDeleteProgram(m_unRenderModelProgramID);
		}
		if (m_unCompanionWindowProgramID)
		{
			glDeleteProgram(m_unCompanionWindowProgramID);
		}
		glDeleteRenderbuffers(1, &leftEyeDesc.m_nDepthBufferId);
		glDeleteTextures(1, &leftEyeDesc.m_nRenderTextureId);
		glDeleteFramebuffers(1, &leftEyeDesc.m_nRenderFramebufferId);
		glDeleteTextures(1, &leftEyeDesc.m_nResolveTextureId);
		glDeleteFramebuffers(1, &leftEyeDesc.m_nResolveFramebufferId);
		glDeleteRenderbuffers(1, &rightEyeDesc.m_nDepthBufferId);
		glDeleteTextures(1, &rightEyeDesc.m_nRenderTextureId);
		glDeleteFramebuffers(1, &rightEyeDesc.m_nRenderFramebufferId);
		glDeleteTextures(1, &rightEyeDesc.m_nResolveTextureId);
		glDeleteFramebuffers(1, &rightEyeDesc.m_nResolveFramebufferId);
		if (m_unCompanionWindowVAO != 0)
		{
			glDeleteVertexArrays(1, &m_unCompanionWindowVAO);
		}
		if (m_unSceneVAO != 0)
		{
			glDeleteVertexArrays(1, &m_unSceneVAO);
		}
		if (m_unControllerVAO != 0)
		{
			glDeleteVertexArrays(1, &m_unControllerVAO);
		}
	}
	if (m_pCompanionWindow)
	{
		SDL_DestroyWindow(m_pCompanionWindow);
		m_pCompanionWindow = NULL;
	}
	SDL_Quit();
}
void CMainApplication::ProcessVREvent(const vr::VREvent_t & event)
{
	switch (event.eventType)
	{
	case vr::VREvent_TrackedDeviceActivated:
	{
		SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
		dprintf("Device %u attached. Setting up render model.\n", event.trackedDeviceIndex);
	}
	break;
	case vr::VREvent_TrackedDeviceDeactivated:
	{
		dprintf("Device %u detached.\n", event.trackedDeviceIndex);
	}
	break;
	case vr::VREvent_TrackedDeviceUpdated:
	{
		dprintf("Device %u updated.\n", event.trackedDeviceIndex);
	}
	break;
	}
}
GLuint CMainApplication::CompileGLShader(const char *pchShaderName, const char *pchVertexShader, const char *pchFragmentShader)
{
	GLuint unProgramID = glCreateProgram();
	GLuint nSceneVertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(nSceneVertexShader, 1, &pchVertexShader, NULL);
	glCompileShader(nSceneVertexShader);
	GLint vShaderCompiled = GL_FALSE;
	glGetShaderiv(nSceneVertexShader, GL_COMPILE_STATUS, &vShaderCompiled);
	if (vShaderCompiled != GL_TRUE)
	{
		dprintf("%s - Unable to compile vertex shader %d!\n", pchShaderName, nSceneVertexShader);
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneVertexShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneVertexShader);
	glDeleteShader(nSceneVertexShader); // the program hangs onto this once it's attached
	GLuint  nSceneFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(nSceneFragmentShader, 1, &pchFragmentShader, NULL);
	glCompileShader(nSceneFragmentShader);
	GLint fShaderCompiled = GL_FALSE;
	glGetShaderiv(nSceneFragmentShader, GL_COMPILE_STATUS, &fShaderCompiled);
	if (fShaderCompiled != GL_TRUE)
	{
		dprintf("%s - Unable to compile fragment shader %d!\n", pchShaderName, nSceneFragmentShader);
		glDeleteProgram(unProgramID);
		glDeleteShader(nSceneFragmentShader);
		return 0;
	}
	glAttachShader(unProgramID, nSceneFragmentShader);
	glDeleteShader(nSceneFragmentShader); // the program hangs onto this once it's attached
	glLinkProgram(unProgramID);
	GLint programSuccess = GL_TRUE;
	glGetProgramiv(unProgramID, GL_LINK_STATUS, &programSuccess);
	if (programSuccess != GL_TRUE)
	{
		dprintf("%s - Error linking program %d!\n", pchShaderName, unProgramID);
		glDeleteProgram(unProgramID);
		return 0;
	}
	glUseProgram(unProgramID);
	glUseProgram(0);
	return unProgramID;
}
bool CMainApplication::CreateAllShaders()
{
	m_unSceneProgramID = CompileGLShader(
		"Scene",
		// Vertex Shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec2 v2UVcoordsIn;\n"
		"layout(location = 2) in vec3 v3NormalIn;\n"
		"out vec2 v2UVcoords;\n"
		"void main()\n"
		"{\n"
		"	v2UVcoords = v2UVcoordsIn;\n"
		"	gl_Position = matrix * position;\n"
		"}\n",
		// Fragment Shader
		"#version 410 core\n"
		"uniform sampler2D mytexture;\n"
		"in vec2 v2UVcoords;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = texture(mytexture, v2UVcoords);\n"
		"}\n"
		);
	m_nSceneMatrixLocation = glGetUniformLocation(m_unSceneProgramID, "matrix");
	if (m_nSceneMatrixLocation == -1)
	{
		dprintf("Unable to find matrix uniform in scene shader\n");
		return false;
	}
	m_unControllerTransformProgramID = CompileGLShader(
		"Controller",
		// vertex shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec3 v3ColorIn;\n"
		"out vec4 v4Color;\n"
		"void main()\n"
		"{\n"
		"	v4Color.xyz = v3ColorIn; v4Color.a = 1.0;\n"
		"	gl_Position = matrix * position;\n"
		"}\n",
		// fragment shader
		"#version 410\n"
		"in vec4 v4Color;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = v4Color;\n"
		"}\n"
		);
	m_nControllerMatrixLocation = glGetUniformLocation(m_unControllerTransformProgramID, "matrix");
	if (m_nControllerMatrixLocation == -1)
	{
		dprintf("Unable to find matrix uniform in controller shader\n");
		return false;
	}
	m_unRenderModelProgramID = CompileGLShader(
		"render model",
		// vertex shader
		"#version 410\n"
		"uniform mat4 matrix;\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec3 v3NormalIn;\n"
		"layout(location = 2) in vec2 v2TexCoordsIn;\n"
		"out vec2 v2TexCoord;\n"
		"void main()\n"
		"{\n"
		"	v2TexCoord = v2TexCoordsIn;\n"
		"	gl_Position = matrix * vec4(position.xyz, 1);\n"
		"}\n",
		//fragment shader
		"#version 410 core\n"
		"uniform sampler2D diffuse;\n"
		"in vec2 v2TexCoord;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"   outputColor = texture( diffuse, v2TexCoord);\n"
		"}\n"
		);
	m_nRenderModelMatrixLocation = glGetUniformLocation(m_unRenderModelProgramID, "matrix");
	if (m_nRenderModelMatrixLocation == -1)
	{
		dprintf("Unable to find matrix uniform in render model shader\n");
		return false;
	}
	m_unCompanionWindowProgramID = CompileGLShader(
		"CompanionWindow",
		// vertex shader
		"#version 410 core\n"
		"layout(location = 0) in vec4 position;\n"
		"layout(location = 1) in vec2 v2UVIn;\n"
		"noperspective out vec2 v2UV;\n"
		"void main()\n"
		"{\n"
		"	v2UV = v2UVIn;\n"
		"	gl_Position = position;\n"
		"}\n",
		// fragment shader
		"#version 410 core\n"
		"uniform sampler2D mytexture;\n"
		"noperspective in vec2 v2UV;\n"
		"out vec4 outputColor;\n"
		"void main()\n"
		"{\n"
		"		outputColor = texture(mytexture, v2UV);\n"
		"}\n"
		);
	return m_unSceneProgramID != 0
		&& m_unControllerTransformProgramID != 0
		&& m_unRenderModelProgramID != 0
		&& m_unCompanionWindowProgramID != 0;
}
bool CMainApplication::SetupTexturemaps()
{
	std::string sExecutableDirectory = Path_StripFilename(Path_GetExecutablePath());
	std::string strFullPath = Path_MakeAbsolute("../cube_texture.png", sExecutableDirectory);
	std::vector<unsigned char> imageRGBA;
	unsigned nImageWidth, nImageHeight;
	unsigned nError = lodepng::decode(imageRGBA, nImageWidth, nImageHeight, strFullPath.c_str());
	if (nError != 0)
		return false;
	glGenTextures(1, &m_iTexture);
	glBindTexture(GL_TEXTURE_2D, m_iTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, nImageWidth, nImageHeight,
		0, GL_RGBA, GL_UNSIGNED_BYTE, &imageRGBA[0]);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	GLfloat fLargest;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);
	glBindTexture(GL_TEXTURE_2D, 0);
	return (m_iTexture != 0);
}
void CMainApplication::AddCubeVertex(float fl0, float fl1, float fl2, float fl3, float fl4, std::vector<float> &vertdata)
{
	vertdata.push_back(fl0);
	vertdata.push_back(fl1);
	vertdata.push_back(fl2);
	vertdata.push_back(fl3);
	vertdata.push_back(fl4);
}
void CMainApplication::AddCubeToScene(Matrix4 mat, std::vector<float> &vertdata)
{
	// Matrix4 mat( outermat.data() );
	Vector4 A = mat * Vector4(0, 0, 0, 1);
	Vector4 B = mat * Vector4(1, 0, 0, 1);
	Vector4 C = mat * Vector4(1, 1, 0, 1);
	Vector4 D = mat * Vector4(0, 1, 0, 1);
	Vector4 E = mat * Vector4(0, 0, 1, 1);
	Vector4 F = mat * Vector4(1, 0, 1, 1);
	Vector4 G = mat * Vector4(1, 1, 1, 1);
	Vector4 H = mat * Vector4(0, 1, 1, 1);
	// triangles instead of quads
	AddCubeVertex(E.x, E.y, E.z, 0, 1, vertdata); //Front
	AddCubeVertex(F.x, F.y, F.z, 1, 1, vertdata);
	AddCubeVertex(G.x, G.y, G.z, 1, 0, vertdata);
	AddCubeVertex(G.x, G.y, G.z, 1, 0, vertdata);
	AddCubeVertex(H.x, H.y, H.z, 0, 0, vertdata);
	AddCubeVertex(E.x, E.y, E.z, 0, 1, vertdata);
	AddCubeVertex(B.x, B.y, B.z, 0, 1, vertdata); //Back
	AddCubeVertex(A.x, A.y, A.z, 1, 1, vertdata);
	AddCubeVertex(D.x, D.y, D.z, 1, 0, vertdata);
	AddCubeVertex(D.x, D.y, D.z, 1, 0, vertdata);
	AddCubeVertex(C.x, C.y, C.z, 0, 0, vertdata);
	AddCubeVertex(B.x, B.y, B.z, 0, 1, vertdata);
	AddCubeVertex(H.x, H.y, H.z, 0, 1, vertdata); //Top
	AddCubeVertex(G.x, G.y, G.z, 1, 1, vertdata);
	AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
	AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
	AddCubeVertex(D.x, D.y, D.z, 0, 0, vertdata);
	AddCubeVertex(H.x, H.y, H.z, 0, 1, vertdata);
	AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata); //Bottom
	AddCubeVertex(B.x, B.y, B.z, 1, 1, vertdata);
	AddCubeVertex(F.x, F.y, F.z, 1, 0, vertdata);
	AddCubeVertex(F.x, F.y, F.z, 1, 0, vertdata);
	AddCubeVertex(E.x, E.y, E.z, 0, 0, vertdata);
	AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata);
	AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata); //Left
	AddCubeVertex(E.x, E.y, E.z, 1, 1, vertdata);
	AddCubeVertex(H.x, H.y, H.z, 1, 0, vertdata);
	AddCubeVertex(H.x, H.y, H.z, 1, 0, vertdata);
	AddCubeVertex(D.x, D.y, D.z, 0, 0, vertdata);
	AddCubeVertex(A.x, A.y, A.z, 0, 1, vertdata);
	AddCubeVertex(F.x, F.y, F.z, 0, 1, vertdata); //Right
	AddCubeVertex(B.x, B.y, B.z, 1, 1, vertdata);
	AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
	AddCubeVertex(C.x, C.y, C.z, 1, 0, vertdata);
	AddCubeVertex(G.x, G.y, G.z, 0, 0, vertdata);
	AddCubeVertex(F.x, F.y, F.z, 0, 1, vertdata);
}
void CMainApplication::SetupCameras()
{
	m_mat4ProjectionLeft = GetHMDMatrixProjectionEye(vr::Eye_Left);
	m_mat4ProjectionRight = GetHMDMatrixProjectionEye(vr::Eye_Right);
	m_mat4eyePosLeft = GetHMDMatrixPoseEye(vr::Eye_Left);
	m_mat4eyePosRight = GetHMDMatrixPoseEye(vr::Eye_Right);
}
bool CMainApplication::CreateFrameBuffer(int nWidth, int nHeight, FramebufferDesc &framebufferDesc)
{
	glGenFramebuffers(1, &framebufferDesc.m_nRenderFramebufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nRenderFramebufferId);
	glGenRenderbuffers(1, &framebufferDesc.m_nDepthBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);
	glRenderbufferStorageMultisample(GL_RENDERBUFFER, 4, GL_DEPTH_COMPONENT, nWidth, nHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, framebufferDesc.m_nDepthBufferId);
	glGenTextures(1, &framebufferDesc.m_nRenderTextureId);
	glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId);
	glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, 4, GL_RGBA8, nWidth, nHeight, true);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, framebufferDesc.m_nRenderTextureId, 0);
	glGenFramebuffers(1, &framebufferDesc.m_nResolveFramebufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferDesc.m_nResolveFramebufferId);
	glGenTextures(1, &framebufferDesc.m_nResolveTextureId);
	glBindTexture(GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, nWidth, nHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, framebufferDesc.m_nResolveTextureId, 0);
	// check FBO status
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
	{
		return false;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return true;
}
bool CMainApplication::SetupStereoRenderTargets()
{
	if (!m_pHMD)
		return false;
	m_pHMD->GetRecommendedRenderTargetSize(&m_nRenderWidth, &m_nRenderHeight);
	CreateFrameBuffer(m_nRenderWidth, m_nRenderHeight, leftEyeDesc);
	CreateFrameBuffer(m_nRenderWidth, m_nRenderHeight, rightEyeDesc);
	return true;
}
void CMainApplication::SetupCompanionWindow()
{
	if (!m_pHMD)
		return;
	std::vector<VertexDataWindow> vVerts;
	
	//上下颠倒修改点
	// left eye verts
	 vVerts.push_back(VertexDataWindow(Vector2(-1, -1), Vector2(0, 0)));
	 vVerts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(1, 0)));
	 vVerts.push_back(VertexDataWindow(Vector2(-1, 1), Vector2(0, 1)));
	 vVerts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(1, 1)));
	// right eye verts
	 vVerts.push_back(VertexDataWindow(Vector2(0, -1), Vector2(0, 0)));
	 vVerts.push_back(VertexDataWindow(Vector2(1, -1), Vector2(1, 0)));
	 vVerts.push_back(VertexDataWindow(Vector2(0, 1), Vector2(0, 1)));
	 vVerts.push_back(VertexDataWindow(Vector2(1, 1), Vector2(1, 1)));


	GLushort vIndices[] = { 0, 1, 3,   0, 3, 2,   4, 5, 7,   4, 7, 6 };
	m_uiCompanionWindowIndexSize = _countof(vIndices);
	glGenVertexArrays(1, &m_unCompanionWindowVAO);
	glBindVertexArray(m_unCompanionWindowVAO);
	glGenBuffers(1, &m_glCompanionWindowIDVertBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_glCompanionWindowIDVertBuffer);
	glBufferData(GL_ARRAY_BUFFER, vVerts.size() * sizeof(VertexDataWindow), &vVerts[0], GL_STATIC_DRAW);
	glGenBuffers(1, &m_glCompanionWindowIDIndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_glCompanionWindowIDIndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_uiCompanionWindowIndexSize * sizeof(GLushort), &vIndices[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, position));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexDataWindow), (void *)offsetof(VertexDataWindow, texCoord));
	glBindVertexArray(0);
	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}
Matrix4 CMainApplication::GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye)
{
	if (!m_pHMD)
		return Matrix4();
	vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix(nEye, m_fNearClip, m_fFarClip);
	return Matrix4(
		mat.m[0][0], mat.m[1][0], mat.m[2][0], mat.m[3][0],
		mat.m[0][1], mat.m[1][1], mat.m[2][1], mat.m[3][1],
		mat.m[0][2], mat.m[1][2], mat.m[2][2], mat.m[3][2],
		mat.m[0][3], mat.m[1][3], mat.m[2][3], mat.m[3][3]
		);
}
Matrix4 CMainApplication::GetHMDMatrixPoseEye(vr::Hmd_Eye nEye)
{
	if (!m_pHMD)
		return Matrix4();
	vr::HmdMatrix34_t matEyeRight = m_pHMD->GetEyeToHeadTransform(nEye);
	Matrix4 matrixObj(
		matEyeRight.m[0][0], matEyeRight.m[1][0], matEyeRight.m[2][0], 0.0,
		matEyeRight.m[0][1], matEyeRight.m[1][1], matEyeRight.m[2][1], 0.0,
		matEyeRight.m[0][2], matEyeRight.m[1][2], matEyeRight.m[2][2], 0.0,
		matEyeRight.m[0][3], matEyeRight.m[1][3], matEyeRight.m[2][3], 1.0f
		);
	return matrixObj.invert();
}
Matrix4 CMainApplication::GetCurrentViewProjectionMatrix(vr::Hmd_Eye nEye)
{
	Matrix4 matMVP;
	if (nEye == vr::Eye_Left)
	{
		matMVP = m_mat4ProjectionLeft * m_mat4eyePosLeft * m_mat4HMDPose;
	}
	else if (nEye == vr::Eye_Right)
	{
		matMVP = m_mat4ProjectionRight * m_mat4eyePosRight *  m_mat4HMDPose;
	}
	return matMVP;
}

CGLRenderModel *CMainApplication::FindOrLoadRenderModel(const char *pchRenderModelName)
{
	CGLRenderModel *pRenderModel = NULL;
	for (std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++)
	{
		if (!stricmp((*i)->GetName().c_str(), pchRenderModelName))
		{
			pRenderModel = *i;
			break;
		}
	}
	// load the model if we didn't find one
	if (!pRenderModel)
	{
		vr::RenderModel_t *pModel;
		vr::EVRRenderModelError error;
		while (1)
		{
			error = vr::VRRenderModels()->LoadRenderModel_Async(pchRenderModelName, &pModel);
			if (error != vr::VRRenderModelError_Loading)
				break;
			ThreadSleep(1);
		}
		if (error != vr::VRRenderModelError_None)
		{
			dprintf("Unable to load render model %s - %s\n", pchRenderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
			return NULL; // move on to the next tracked device
		}
		vr::RenderModel_TextureMap_t *pTexture;
		while (1)
		{
			error = vr::VRRenderModels()->LoadTexture_Async(pModel->diffuseTextureId, &pTexture);
			if (error != vr::VRRenderModelError_Loading)
				break;
			ThreadSleep(1);
		}
		if (error != vr::VRRenderModelError_None)
		{
			dprintf("Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, pchRenderModelName);
			vr::VRRenderModels()->FreeRenderModel(pModel);
			return NULL; // move on to the next tracked device
		}
		pRenderModel = new CGLRenderModel(pchRenderModelName);
		if (!pRenderModel->BInit(*pModel, *pTexture))
		{
			dprintf("Unable to create GL model from render model %s\n", pchRenderModelName);
			delete pRenderModel;
			pRenderModel = NULL;
		}
		else
		{
			m_vecRenderModels.push_back(pRenderModel);
		}
		vr::VRRenderModels()->FreeRenderModel(pModel);
		vr::VRRenderModels()->FreeTexture(pTexture);
	}
	return pRenderModel;
}
void CMainApplication::SetupRenderModelForTrackedDevice(vr::TrackedDeviceIndex_t unTrackedDeviceIndex)
{
	if (unTrackedDeviceIndex >= vr::k_unMaxTrackedDeviceCount)
		return;
	// try to find a model we've already set up
	std::string sRenderModelName = GetTrackedDeviceString(m_pHMD, unTrackedDeviceIndex, vr::Prop_RenderModelName_String);
	CGLRenderModel *pRenderModel = FindOrLoadRenderModel(sRenderModelName.c_str());
	if (!pRenderModel)
	{
		std::string sTrackingSystemName = GetTrackedDeviceString(m_pHMD, unTrackedDeviceIndex, vr::Prop_TrackingSystemName_String);
		dprintf("Unable to load render model for tracked device %d (%s.%s)", unTrackedDeviceIndex, sTrackingSystemName.c_str(), sRenderModelName.c_str());
	}
	else
	{
		m_rTrackedDeviceToRenderModel[unTrackedDeviceIndex] = pRenderModel;
		m_rbShowTrackedDevice[unTrackedDeviceIndex] = true;
	}
}
void CMainApplication::SetupRenderModels()
{
	memset(m_rTrackedDeviceToRenderModel, 0, sizeof(m_rTrackedDeviceToRenderModel));
	if (!m_pHMD)
		return;
	for (uint32_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
	{
		if (!m_pHMD->IsTrackedDeviceConnected(unTrackedDevice))
			continue;
		SetupRenderModelForTrackedDevice(unTrackedDevice);
	}
}
Matrix4 CMainApplication::ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t &matPose)
{
	Matrix4 matrixObj(
		matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
		matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
		matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
		matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
		);
	return matrixObj;
}
CGLRenderModel::CGLRenderModel(const std::string & sRenderModelName)
	: m_sModelName(sRenderModelName)
{
	m_glIndexBuffer = 0;
	m_glVertArray = 0;
	m_glVertBuffer = 0;
	m_glTexture = 0;
}
CGLRenderModel::~CGLRenderModel()
{
	Cleanup();
}
bool CGLRenderModel::BInit(const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture)
{
	// create and bind a VAO to hold state for this model
	glGenVertexArrays(1, &m_glVertArray);
	glBindVertexArray(m_glVertArray);
	// Populate a vertex buffer
	glGenBuffers(1, &m_glVertBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_glVertBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vr::RenderModel_Vertex_t) * vrModel.unVertexCount, vrModel.rVertexData, GL_STATIC_DRAW);
	// Identify the components in the vertex buffer
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t), (void *)offsetof(vr::RenderModel_Vertex_t, vPosition));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t), (void *)offsetof(vr::RenderModel_Vertex_t, vNormal));
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(vr::RenderModel_Vertex_t), (void *)offsetof(vr::RenderModel_Vertex_t, rfTextureCoord));
	// Create and populate the index buffer
	glGenBuffers(1, &m_glIndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_glIndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint16_t) * vrModel.unTriangleCount * 3, vrModel.rIndexData, GL_STATIC_DRAW);
	glBindVertexArray(0);
	// create and populate the texture
	glGenTextures(1, &m_glTexture);
	glBindTexture(GL_TEXTURE_2D, m_glTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTexture.unWidth, vrDiffuseTexture.unHeight,
		0, GL_RGBA, GL_UNSIGNED_BYTE, vrDiffuseTexture.rubTextureMapData);
	// If this renders black ask McJohn what's wrong.
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	GLfloat fLargest;
	glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);
	glBindTexture(GL_TEXTURE_2D, 0);
	m_unVertexCount = vrModel.unTriangleCount * 3;
	return true;
}
void CGLRenderModel::Cleanup()
{
	if (m_glVertBuffer)
	{
		glDeleteBuffers(1, &m_glIndexBuffer);
		glDeleteVertexArrays(1, &m_glVertArray);
		glDeleteBuffers(1, &m_glVertBuffer);
		m_glIndexBuffer = 0;
		m_glVertArray = 0;
		m_glVertBuffer = 0;
	}
}
/************主函数******************/
int main(int argc, char *argv[])
{
	//cvSetData(yimg, pYUV420, nWidth);
	//cv::namedWindow("cam0", CV_WINDOW_AUTOSIZE);
	Mbuff1 = Mbuff2 = 2;
	//初始化 ffmpeg读取音频
	//AudioInit();
	//opencv 读取视频帧
	//capture = cvCreateFileCapture(VideoAddr);
	QueryPerformanceFrequency(&nFreq);
	//mm.setDefaultAllocator(cv::cuda::HostMem::getAllocator(cv::cuda::HostMem::AllocType::PAGE_LOCKED));
	//mm.create(mm.rows, mm.cols, CV_8UC3);
	//初始化编码器
	
	//NewMatInit();
	MatInit();

	//rate = (double)cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	//printf("原始帧率 %f \n", rate);

	thread_loadframe = SDL_CreateThread(LoadFrame, "LoadFrame", &xx);
	thread_remap = SDL_CreateThread(MatRemap, "MatRemap", &xx);
	//thread_audio = SDL_CreateThread(AudioPlay, "AudioPlay", &xx);

	CMainApplication *pMainApplication = new CMainApplication(argc, argv);
	if (!pMainApplication->BInit())
	{
		pMainApplication->Shutdown();
		return 1;
	}
	pMainApplication->RunMainLoop();
	//AudioEnd();
	pMainApplication->Shutdown();
	return 0;
}