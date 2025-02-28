#pragma once
#include <cstdio>
#include <string>
#include <cstring>

#ifdef _WIN32
#define NOMINMAX
#define NOGDI
#include <WinSock2.h>
#include <windows.h>
#else  // linux
#include <stdlib.h>    
#include <unistd.h>    
#include <sys/types.h>  
#include <sys/stat.h>   
#include <sys/ioctl.h>
#include <fcntl.h>      
#include <termios.h>  
#include <iostream>
#include <errno.h>
#endif // _WIN32


class SerialPort
{
public:
    SerialPort()  {}
    ~SerialPort() {}
    /**
    *	@Brief:		打开串口,成功返回true，失败返回false
    *	@Param		portname(串口名): 在Windows下是"COM1""COM2"等，在Linux下是"/dev/ttyS1"等
    *	@Param		baudrate(波特率): 9600、19200、38400、43000、56000、57600、115200
    *	@Param		parity(校验位): 0为无校验，1为奇校验，2为偶校验，3为标记校验（仅适用于windows)
    *	@Param		databit(数据位): 4-8(windows),5-8(linux)，通常为8位
    *	@Param		stopbit(停止位): 1为1位停止位，2为2位停止位,3为1.5位停止位
    *	@Param		synchronizeflag(同步、异步,仅适用与windows): 0为异步，1为同步
    *	@Return:	真或假
    */
    bool OpenPort(const std::string& portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag = 1);

    //发送数据或写数据，成功返回发送数据长度，失败返回0
    int m_Send(const void* buf, int len);

    //接受数据或读数据，成功返回读取实际数据的长度，失败返回0
    int m_Receive(void* buf, int maxlen);

    //获取缓冲区字节数
    inline int getBufferBytes() const {
        int bytesAvailable = 0;
        if(ioctl(pHandle[0], FIONREAD, &bytesAvailable) == -1){
            std::cerr << "Error getting the byte count from buffer." << std::endl;
        }
        return bytesAvailable;
    }

    void Close() { m_Close(); }


private:
    int pHandle[16];
    char synchronizeflag;

    bool m_Open(const char* portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag = 1);
    //关闭串口，参数待定
    void m_Close();
};

#ifdef _WIN32
inline bool SerialPort::m_Open(const char* portname,
    int baudrate,
    char parity,
    char databit,
    char stopbit,
    char synchronizeflag)
{
    this->synchronizeflag = synchronizeflag;
    HANDLE hCom = NULL;

    if (this->synchronizeflag)
    {
        //同步方式
        hCom = CreateFileA(portname, //串口名
            GENERIC_READ | GENERIC_WRITE, //支持读写
            0, //独占方式，串口不支持共享
            NULL,//安全属性指针，默认值为NULL
            OPEN_EXISTING, //打开现有的串口文件
            0, //0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
            NULL); //用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
    }
    else
    {
        //异步方式
        hCom = CreateFileA(portname, //串口名
            GENERIC_READ | GENERIC_WRITE, //支持读写
            0, //独占方式，串口不支持共享
            NULL,//安全属性指针，默认值为NULL
            OPEN_EXISTING, //打开现有的串口文件
            FILE_FLAG_OVERLAPPED, //0：同步方式，FILE_FLAG_OVERLAPPED：异步方式
            NULL); //用于复制文件句柄，默认值为NULL，对串口而言该参数必须置为NULL
    }

    if (hCom == (HANDLE)-1)
    {
        return false;
    }

    //配置缓冲区大小
    if (!SetupComm(hCom, 1024, 1024))
    {
        return false;
    }

    // 配置参数
    DCB p;
    memset(&p, 0, sizeof(p));
    p.DCBlength = sizeof(p);
    p.BaudRate = baudrate; // 波特率
    p.ByteSize = databit; // 数据位

    switch (parity) //校验位
    {
    case 0:
        p.Parity = NOPARITY; //无校验
        break;

    case 1:
        p.Parity = ODDPARITY; //奇校验
        break;

    case 2:
        p.Parity = EVENPARITY; //偶校验
        break;

    case 3:
        p.Parity = MARKPARITY; //标记校验
        break;
    }

    switch (stopbit) //停止位
    {
    case 1:
        p.StopBits = ONESTOPBIT; //1位停止位
        break;

    case 2:
        p.StopBits = TWOSTOPBITS; //2位停止位
        break;

    case 3:
        p.StopBits = ONE5STOPBITS; //1.5位停止位
        break;
    }

    if (!SetCommState(hCom, &p))
    {
        // 设置参数失败
        return false;
    }

    //超时处理,单位：毫秒
    //总超时＝时间系数×读或写的字符数＋时间常量
    COMMTIMEOUTS TimeOuts;
    TimeOuts.ReadIntervalTimeout = 1000; //读间隔超时
    TimeOuts.ReadTotalTimeoutMultiplier = 500; //读时间系数
    TimeOuts.ReadTotalTimeoutConstant = 6500; //读时间常量
    TimeOuts.WriteTotalTimeoutMultiplier = 500; // 写时间系数
    TimeOuts.WriteTotalTimeoutConstant = 2000; //写时间常量
    SetCommTimeouts(hCom, &TimeOuts);

    PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR); //清空串口缓冲区

    memcpy(pHandle, &hCom, sizeof(hCom)); // 保存句柄

    return true;
}

inline void SerialPort::m_Close()
{
    HANDLE hCom = *(HANDLE*)pHandle;
    CloseHandle(hCom);
}

int SerialPort::m_Send(const void* buf, int len)
{
    HANDLE hCom = *(HANDLE*)pHandle;

    if (this->synchronizeflag)
    {
        // 同步方式
        DWORD dwBytesWrite = len; //成功写入的数据字节数
        BOOL bWriteStat = WriteFile(hCom, //串口句柄
            buf, //数据首地址
            dwBytesWrite, //要发送的数据字节数
            &dwBytesWrite, //DWORD*，用来接收返回成功发送的数据字节数
            NULL); //NULL为同步发送，OVERLAPPED*为异步发送

        if (!bWriteStat)
        {
            return 0;
        }

        return dwBytesWrite;
    }
    else
    {
        //异步方式
        OVERLAPPED osWrite = { 0 };
        DWORD dwWritten, dwToWrite = len;
        DWORD dwRes;
        BOOL fRes;

        // Create this write operation's OVERLAPPED structure's hEvent.
        osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        if (osWrite.hEvent == NULL)
            // error creating overlapped event handle
            return FALSE;

        // Issue write.
        if (!WriteFile(hCom, buf, dwToWrite, &dwWritten, &osWrite))
        {
            if (GetLastError() != ERROR_IO_PENDING) {
                // WriteFile failed, but isn't delayed. Report error and abort.
                fRes = FALSE;
            }
            else
                // Write is pending.
                dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
            switch (dwRes)
            {
                // OVERLAPPED structure's event has been signaled. 
            case WAIT_OBJECT_0:
                if (!GetOverlappedResult(hCom, &osWrite, &dwWritten, FALSE))
                    fRes = FALSE;
                else
                    // Write operation completed successfully.
                    fRes = TRUE;
                break;

            default:
                // An error has occurred in WaitForSingleObject.
                // This usually indicates a problem with the
               // OVERLAPPED structure's event handle.
                fRes = FALSE;
                break;
            }
        }
        else
            // WriteFile completed immediately.
            fRes = TRUE;
        CloseHandle(osWrite.hEvent);
        return dwWritten;
    }
}

inline int SerialPort::m_Receive(void* buf, int maxlen)
{
    HANDLE hCom = *(HANDLE*)pHandle;
    if (this->synchronizeflag)
    {
        //同步方式
        DWORD wCount = maxlen; //成功读取的数据字节数
        BOOL bReadStat = ReadFile(hCom, //串口句柄
            buf, //数据首地址
            wCount, //要读取的数据最大字节数
            &wCount, //DWORD*,用来接收返回成功读取的数据字节数
            NULL); //NULL为同步发送，OVERLAPPED*为异步发送
        if (!bReadStat)
        {
            return 0;
        }

        return wCount;
    }
    else
    {
        //异步方式
        DWORD wCount = maxlen; //成功读取的数据字节数
        DWORD dwErrorFlags; //错误标志
        COMSTAT comStat; //通讯状态
        OVERLAPPED m_osRead; //异步输入输出结构体

        //创建一个用于OVERLAPPED的事件处理，不会真正用到，但系统要求这么做
        memset(&m_osRead, 0, sizeof(m_osRead));
        m_osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, "ReadEvent");

        ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误，获得设备当前状态

        if (!comStat.cbInQue)return 0; //如果输入缓冲区字节数为0，则返回false

        BOOL bReadStat = ReadFile(hCom, //串口句柄
            buf, //数据首地址
            wCount, //要读取的数据最大字节数
            &wCount, //DWORD*,用来接收返回成功读取的数据字节数
            &m_osRead); //NULL为同步发送，OVERLAPPED*为异步发送
        if (!bReadStat)
        {
            if (GetLastError() == ERROR_IO_PENDING) //如果串口正在读取中
            {
                //GetOverlappedResult函数的最后一个参数设为TRUE
                //函数会一直等待，直到读操作完成或由于错误而返回
                GetOverlappedResult(hCom, &m_osRead, &wCount, TRUE);
            }
            else
            {
                ClearCommError(hCom, &dwErrorFlags, &comStat); //清除通讯错误
                CloseHandle(m_osRead.hEvent); //关闭并释放hEvent的内存
                return 0;
            }
        }

        return wCount;
    }
}

#else //linux
inline bool SerialPort::m_Open(const char* portname,
    int baudrate,
    char parity,
    char databit,
    char stopbit,
    char synchronizeflag)
{
    // 打开串口
    pHandle[0] = -1;
    // 以 读写、不阻塞 方式打开
    pHandle[0] = ::open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK);

    // 打开失败，则打印失败信息，返回false
    if (pHandle[0] == -1)
    {
        std::cout << portname << " open failed , may be you need 'sudo' permission." << std::endl;
        return false;
    }

    // 设置串口参数
    // 创建串口参数对象
    struct termios options;
    // 先获得串口的当前参数
    if (tcgetattr(pHandle[0], &options) < 0)
    {
        std::cout << portname << " open failed , get serial port attributes failed." << std::endl;
        return false;
    }

    // 设置波特率
    switch (baudrate)
    {
    case 4800:
        cfsetispeed(&options, B4800);
        cfsetospeed(&options, B4800);
        break;
    case 9600:
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        break;
    case 19200:
        cfsetispeed(&options, B19200);
        cfsetospeed(&options, B19200);
        break;
    case 38400:
        cfsetispeed(&options, B38400);
        cfsetospeed(&options, B38400);
        break;
    case 57600:
        cfsetispeed(&options, B57600);
        cfsetospeed(&options, B57600);
        break;
    case 115200:
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        break;
    default:
        std::cout << portname << " open failed , unkown baudrate , only support 4800,9600,19200,38400,57600,115200." << std::endl;
        return false;
    }

    // 设置校验位
    switch (parity)
    {
        // 无校验
    case 0:
        options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
        break;
        // 设置奇校验
    case 1:
        options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
        options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
        options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
        // 设置偶校验
    case 2:
        options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
        options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
        options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
        options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
        break;
    default:
        std::cout << portname << " open failed , unkown parity ." << std::endl;
        return false;
    }

    // 设置数据位
    switch (databit)
    {
    case 5:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag &= ~CSIZE;//屏蔽其它标志位
        options.c_cflag |= CS8;
        break;
    default:
        std::cout << portname << " open failed , unkown databit ." << std::endl;
        return false;
    }

    // 设置停止位
    switch (stopbit)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;//CSTOPB：使用1位停止位
        break;
    case 2:
        options.c_cflag |= CSTOPB;//CSTOPB：使用2位停止位
        break;
    default:
        std::cout << portname << " open failed , unkown stopbit ." << std::endl;
        return false;
    }

    // 设置原始输入模式
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
        | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_iflag &= ~(ICRNL | IGNCR);

    // 激活新配置
    if ((tcsetattr(pHandle[0], TCSANOW, &options)) != 0)
    {
        std::cout << portname << " open failed , can not complete set attributes ." << std::endl;
        return false;
    }

    return true;
}

inline void SerialPort::m_Close()
{
    if (pHandle[0] != -1)
    {
        ::close(pHandle[0]);
    }
}

inline int SerialPort::m_Send(const void* buf, int len)
{
    int sendCount = 0;
    if (pHandle[0] != -1)
    {
        // 将 buf 和 len 转换成api要求的格式
        const char* buffer = (char*)buf;
        size_t length = len;
        // 已写入的数据个数
        ssize_t tmp;

        while (length > 0)
        {
            if ((tmp = write(pHandle[0], buffer, length)) <= 0)
            {
                if (tmp < 0 && errno == EINTR)
                {
                    tmp = 0;
                }
                else
                {
                    break;
                }
            }
            length -= tmp;
            buffer += tmp;
        }

        sendCount = len - length;
    }

    return sendCount;
}

inline int SerialPort::m_Receive(void* buf, int maxlen)
{
    int receiveCount = ::read(pHandle[0], buf, maxlen);
    if (receiveCount < 0)
    {
        receiveCount = 0;
    }
    return receiveCount;
}
#endif // _WIN32 

inline bool SerialPort::OpenPort(const std::string& portname, int baudrate, char parity, char databit, char stopbit, char synchronizeflag)
{
    return m_Open(portname.c_str(), baudrate, parity, databit, stopbit, synchronizeflag);
}
