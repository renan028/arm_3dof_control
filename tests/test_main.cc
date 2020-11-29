#include <gtest/gtest.h>
#include "test_fk.h"
#include "test_ik.h"
#include "test_angle.h"
#include "test_trajectory.h"
#include "test_data_convert.h"
#include "test_json_parser.h"

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}