#include "pch.h"
#include "helpers/TestSerialInterface.h"
#include "helpers/TestClock.h"
#include "helpers/modbus_helpers.h"
#include "actuator.h"

using namespace orcaSDK;

namespace {

std::vector<char> with_crc(std::deque<char> message)
{
	ModbusTesting::CalculateAndAppendCRC(message);
	return { message.begin(), message.end() };
}

std::deque<char> write_response(uint16_t address, uint16_t width)
{
	std::deque<char> response{
		'\x1', '\x10',
		char(address >> 8), char(address),
		char(width >> 8), char(width)
	};
	ModbusTesting::CalculateAndAppendCRC(response);
	return response;
}

}

class AsyncHapticTests : public testing::Test
{
protected:
	AsyncHapticTests() :
		serial_interface(std::make_shared<TestSerialInterface>()),
		clock(std::make_shared<TestClock>()),
		motor(serial_interface, clock, "unimportant")
	{}

	std::shared_ptr<TestSerialInterface> serial_interface;
	std::shared_ptr<TestClock> clock;
	Actuator motor;
};

TEST_F(AsyncHapticTests, AsyncSettersQueueWithoutSendingOrBlockingForAResponse)
{
	motor.set_constant_force_async(1);
	motor.set_friction_async(2);
	motor.set_damping_async(3);
	motor.set_spring_effect_async(0, 4, 5);

	EXPECT_TRUE(serial_interface->sendBuffer.empty());
	EXPECT_EQ(0, serial_interface->blocking_receive_count);
	EXPECT_EQ(5, motor.modbus_client.get_queue_size());
}

TEST_F(AsyncHapticTests, SpringAsyncUsesBlockingSetterRegistersAndEncoding)
{
	motor.set_spring_effect_async(
		1, 0x1234, 0x12345678, 0x9ABC, 0xDEF0, Actuator::SpringCoupling::negative);
	motor.run_out();

	std::vector<char> expected = with_crc({
		'\x1', '\x10', '\x2', char(0x8A), '\x0', '\x6', '\xC',
		'\x12', '\x34', '\x56', '\x78', '\x12', '\x34',
		'\x0', '\x2', char(0x9A), char(0xBC), char(0xDE), char(0xF0)
	});
	EXPECT_EQ(expected, serial_interface->sendBuffer);
}

TEST_F(AsyncHapticTests, SpringUpdatesStayBoundedIndependentlyForEachSpring)
{
	for (int32_t value = 1; value <= 1000; value++) {
		motor.set_spring_effect_async(0, uint16_t(value), value);
		motor.set_spring_effect_async(1, uint16_t(value), value);
		motor.set_spring_effect_async(2, uint16_t(value), value);
	}

	EXPECT_EQ(3, motor.modbus_client.get_queue_size());
}

TEST_F(AsyncHapticTests, ConstantForceAsyncUsesBlockingSetterRegisterAndEncoding)
{
	motor.set_constant_force_async(0x12345678);
	motor.run_out();

	std::vector<char> expected = with_crc({
		'\x1', '\x10', '\x2', '\x82', '\x0', '\x2', '\x4',
		'\x56', '\x78', '\x12', '\x34'
	});
	EXPECT_EQ(expected, serial_interface->sendBuffer);
}

TEST_F(AsyncHapticTests, FrictionAsyncUsesBothExistingSignedWideRegisterWrites)
{
	motor.set_friction_async(0x12345678);
	motor.run_out();

	std::vector<char> expected_forward = with_crc({
		'\x1', '\x10', '\x2', char(0xA2), '\x0', '\x2', '\x4',
		'\x56', '\x78', '\x12', '\x34'
	});
	EXPECT_EQ(expected_forward, serial_interface->sendBuffer);

	serial_interface->consume_new_message(write_response(674, 2));
	motor.run_in();
	clock->pass_time(2001);
	serial_interface->sendBuffer.clear();
	motor.run_out();

	std::vector<char> expected_reverse = with_crc({
		'\x1', '\x10', '\x2', char(0xA4), '\x0', '\x2', '\x4',
		'\x56', '\x78', '\x12', '\x34'
	});
	EXPECT_EQ(expected_reverse, serial_interface->sendBuffer);
}

TEST_F(AsyncHapticTests, DampingAsyncUsesBlockingSetterRegisterAndEncoding)
{
	motor.set_damping_async(0x1234);
	motor.run_out();

	std::vector<char> expected = with_crc({
		'\x1', '\x6', '\x2', '\x96', '\x12', '\x34'
	});
	EXPECT_EQ(expected, serial_interface->sendBuffer);
}

TEST_F(AsyncHapticTests, RepeatedUnsentUpdatesStayBoundedAndKeepOnlyLatestValues)
{
	for (int32_t value = 1; value <= 1000; value++) {
		motor.set_constant_force_async(value);
		motor.set_friction_async(value);
		motor.set_damping_async(uint16_t(value));
	}

	EXPECT_EQ(4, motor.modbus_client.get_queue_size());

	motor.run_out();
	std::vector<char> expected_latest_force = with_crc({
		'\x1', '\x10', '\x2', '\x82', '\x0', '\x2', '\x4',
		char(0x03), char(0xE8), '\x0', '\x0'
	});
	EXPECT_EQ(expected_latest_force, serial_interface->sendBuffer);
}

TEST_F(AsyncHapticTests, LatestUpdatesDoNotDisplaceAnActiveWriteButReplaceAllUnsentWrites)
{
	motor.set_constant_force_async(1);
	motor.set_friction_async(1);
	motor.set_damping_async(1);
	clock->pass_time(2001);
	motor.run_out();

	for (int32_t value = 2; value <= 1000; value++) {
		motor.set_constant_force_async(value);
		motor.set_friction_async(value);
		motor.set_damping_async(uint16_t(value));
	}

	EXPECT_EQ(5, motor.modbus_client.get_queue_size());
}

TEST_F(AsyncHapticTests, BlockingHapticSettersStillWaitForAndProcessResponses)
{
	std::deque<char> constant_force_response = write_response(CONSTANT_FORCE_MN, 2);
	serial_interface->consume_new_message(constant_force_response);
	EXPECT_FALSE(motor.set_constant_force(1234));

	std::deque<char> damping_response{
		'\x1', '\x6', '\x2', '\x96', '\x4', '\xD2'
	};
	ModbusTesting::CalculateAndAppendCRC(damping_response);
	serial_interface->consume_new_message(damping_response);
	EXPECT_FALSE(motor.set_damper(1234));

	EXPECT_EQ(2, serial_interface->blocking_receive_count);
	EXPECT_EQ(0, motor.modbus_client.get_queue_size());
}

TEST_F(AsyncHapticTests, HapticCommandStreamingContinuesAndUpdatesPositionFeedback)
{
	std::deque<char> mode_responses{
		'\x1', '\x6', '\x0', '\x3', '\x0', '\x4'
	};
	ModbusTesting::CalculateAndAppendCRC(mode_responses);
	std::deque<char> mode_read{ '\x1', '\x3', '\x2', '\x0', '\x4' };
	ModbusTesting::CalculateAndAppendCRC(mode_read);
	mode_responses.insert(mode_responses.end(), mode_read.begin(), mode_read.end());
	serial_interface->consume_new_message(mode_responses);
	ASSERT_FALSE(motor.set_mode(MotorMode::HapticMode));
	serial_interface->sendBuffer.clear();

	motor.enable_stream();
	motor.set_constant_force_async(1);
	motor.set_friction_async(1);
	motor.set_damping_async(1);
	clock->pass_time(2001);
	motor.run_out();

	motor.set_constant_force_async(2);
	motor.set_friction_async(2);
	motor.set_damping_async(2);
	EXPECT_EQ(6, motor.modbus_client.get_queue_size());

	serial_interface->consume_new_message(write_response(CONSTANT_FORCE_MN, 2));
	motor.run_in();
	clock->pass_time(2001);
	serial_interface->sendBuffer.clear();
	motor.run_out();
	ASSERT_GE(serial_interface->sendBuffer.size(), 3);
	EXPECT_EQ(char(0x64), serial_interface->sendBuffer[1]);
	EXPECT_EQ(char(0x22), serial_interface->sendBuffer[2]);

	std::deque<char> stream_response{
		'\x1', '\x64',
		'\x0', '\x1', '\x0', '\x2',
		'\x0', '\x3', '\x0', '\x4',
		'\x0', '\x5', '\x6', '\x0', '\x7', '\x0', '\x0'
	};
	ModbusTesting::CalculateAndAppendCRC(stream_response);
	serial_interface->consume_new_message(stream_response);
	motor.run_in();

	EXPECT_EQ(65538, motor.stream_cache.position);
	EXPECT_EQ(196612, motor.stream_cache.force);
	EXPECT_EQ(5, motor.stream_cache.power);
	EXPECT_EQ(6, motor.stream_cache.temperature);
	EXPECT_EQ(7, motor.stream_cache.voltage);
	EXPECT_EQ(0, motor.stream_cache.errors);
}
