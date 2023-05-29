#ifndef AR0330_H
#define AR0330_H

struct clk;
struct v4l2_subdev;

struct ar0330_platform_data {
	struct clk *clock;
	unsigned int reset;
};

#endif
