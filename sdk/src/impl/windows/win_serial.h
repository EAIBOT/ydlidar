#if defined(_WIN32)

#ifndef SERIAL_IMPL_WINDOWS_H
#define SERIAL_IMPL_WINDOWS_H

#include "serial.h"
#include "time.h"
#include "windows.h"

namespace serial {

	using std::string;
	using std::wstring;
	using std::invalid_argument;


	class serial::Serial::SerialImpl {
	public:
		SerialImpl (const string &port,
			unsigned long baudrate,
			bytesize_t bytesize,
			parity_t parity,
			stopbits_t stopbits,
			flowcontrol_t flowcontrol);

		virtual ~SerialImpl ();

		bool open ();

		void close ();

		bool isOpen () const;

		size_t available ();

		bool waitReadable (uint32_t timeout);

		void waitByteTimes (size_t count);

		size_t waitfordata(size_t data_count, uint32_t timeout, size_t * returned_size);

		size_t read (uint8_t *buf, size_t size = 1);

		size_t write (const uint8_t *data, size_t length);

		void flush ();

		void flushInput ();

		void flushOutput ();

		void sendBreak (int duration);

		bool setBreak (bool level);

		bool setRTS (bool level);

		bool setDTR (bool level);

		bool waitForChange ();

		bool getCTS ();

		bool getDSR ();

		bool getRI ();

		bool getCD ();

		void setPort (const string &port);

		string getPort () const;

		void setTimeout (Timeout &timeout);

		Timeout getTimeout () const;

		bool setBaudrate (unsigned long baudrate);

		unsigned long getBaudrate () const;

		bool setBytesize (bytesize_t bytesize);

		bytesize_t getBytesize () const;

		bool setParity (parity_t parity);

		parity_t getParity () const;

		bool setStopbits (stopbits_t stopbits);

		stopbits_t getStopbits () const;

		bool setFlowcontrol (flowcontrol_t flowcontrol);

		flowcontrol_t getFlowcontrol () const;


		bool  setDcb(DCB *dcb);


		bool  getDcb(DCB *dcb);

		int readLock ();

		int readUnlock ();

		int writeLock ();

		int writeUnlock ();

	protected:
		bool reconfigurePort ();

	private:
		wstring port_;               // Path to the file descriptor
		HANDLE fd_;
		OVERLAPPED _wait_o;

		bool is_open_;

		Timeout timeout_;           // Timeout for read operations
		unsigned long baudrate_;    // Baudrate

		parity_t parity_;           // Parity
		bytesize_t bytesize_;       // Size of the bytes
		stopbits_t stopbits_;       // Stop Bits
		flowcontrol_t flowcontrol_; // Flow Control

		// Mutex used to lock the read functions
		HANDLE read_mutex;
		// Mutex used to lock the write functions
		HANDLE write_mutex;
	};

}

#endif // SERIAL_IMPL_WINDOWS_H

#endif // if defined(_WIN32)
