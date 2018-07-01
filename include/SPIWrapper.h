#pragma once

#include <cstring>
#include <driver/spi_master.h>

namespace m2d
{
namespace ESP32
{
	class SPITransaction
	{
	private:
		spi_transaction_t _raw_transaction;

	public:
		SPITransaction()
		{
			std::memset(&this->_raw_transaction, 0, sizeof(this->_raw_transaction));
		}

		void set_tx_buffer(const uint8_t *buffer, int size)
		{
			this->_raw_transaction.tx_buffer = buffer;
			this->_raw_transaction.length = size * 8;
		}

		void set_tx_data(const uint8_t *data, int size)
		{
			if (size > 4) {
				return;
			}

			for (int i = 0; i < size; i++) {
				this->_raw_transaction.tx_data[i] = data[i];
			}
			this->_raw_transaction.length = size;
			this->_raw_transaction.flags |= SPI_TRANS_USE_TXDATA;
		}

		void set_rx_buffer(uint8_t *buffer)
		{
			this->_raw_transaction.rx_buffer = buffer;
		}

		void enable_rx_data()
		{
			this->_raw_transaction.flags |= SPI_TRANS_USE_RXDATA;
		}

		void disable_rx_data()
		{
			this->_raw_transaction.flags ^= SPI_TRANS_USE_RXDATA;
		}

		spi_transaction_t *raw_transaction()
		{
			return &this->_raw_transaction;
		}
	};

	class SPIWrapper
	{
	private:
		static void spi_pre_transfer_callback(spi_transaction_t *t) {}
		static void spi_post_transfer_callback(spi_transaction_t *t) {}
		spi_device_handle_t spi;

	public:
		SPIWrapper(int clock_hz, int spi_mode, gpio_num_t sclk, gpio_num_t miso, gpio_num_t mosi, gpio_num_t cs, spi_host_device_t host = HSPI_HOST, uint8_t flags = 0)
		{
			spi_bus_config_t bus_config;
			bus_config.mosi_io_num = mosi;
			bus_config.miso_io_num = miso;
			bus_config.sclk_io_num = sclk;
			bus_config.quadwp_io_num = -1; // Not used
			bus_config.quadhd_io_num = -1; // Not used
			bus_config.max_transfer_sz = 0; // 0 means use default.

			spi_device_interface_config_t device_config;
			device_config.address_bits = 0;
			device_config.command_bits = 0;
			device_config.dummy_bits = 0;
			device_config.mode = spi_mode;
			device_config.duty_cycle_pos = 0;
			device_config.cs_ena_posttrans = 0;
			device_config.cs_ena_pretrans = 0;
			device_config.clock_speed_hz = clock_hz;
			device_config.spics_io_num = cs;
			device_config.flags = flags;
			device_config.queue_size = 1;
			device_config.pre_cb = &SPIWrapper::spi_pre_transfer_callback;
			device_config.post_cb = &SPIWrapper::spi_post_transfer_callback;

			assert(spi_bus_initialize(host, &bus_config, host) == ESP_OK);
			assert(spi_bus_add_device(host, &device_config, &spi) == ESP_OK);
		}

		esp_err_t detach()
		{
			return spi_bus_remove_device(this->spi);
		}

		esp_err_t transmit(SPITransaction transaction)
		{
			return spi_device_transmit(this->spi, transaction.raw_transaction());
		}
	};
}
}
