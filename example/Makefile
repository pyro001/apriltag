CC = gcc
CXX = g++

CPPFLAGS = -I.. `pkg-config --cflags opencv4`
CFLAGS = -g -std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -O3
CXXFLAGS = -g -Wall -O3
LDFLAGS = -lpthread -lm

TARGETS := apriltag_demo  base_demo

.PHONY: all
all: apriltag_demo  base_demo

apriltag_demo: apriltag_demo.o ../Src/libapriltag.a
	@echo "   [$@]"
	@$(CC) -o $@ $^ $(LDFLAGS)

# opencv_demo: opencv_demo.o ../libapriltag.a
# 	@echo "   [$@]"
# 	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv4`

base_demo: base_demo.o ../Src/libapriltag.a
	@echo "   [$@]"
	@$(CXX) -o $@ $^ $(LDFLAGS) `pkg-config --libs opencv4`
%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS) $(CPPFLAGS)

%.o: %.cc
	@echo "   $@"
	@$(CXX) -o $@ -c $< $(CXXFLAGS) $(CPPFLAGS)

.PHONY: clean
clean:
	@rm -rf *.o $(TARGETS)
