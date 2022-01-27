################################################################
#   BSD LICENSE
# 
#   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
#  version: QAT1.7.L.4.6.0-00025
################################################################
ifndef ICP_ROOT 
$(error ICP_ROOT is undefined. Please set the path to upstream repo directory \
	"-> setenv ICP_ROOT <path>")
endif
buildir = ./build
USE_LEGACY_TRANSPORTCTL=y

ifdef UBUNTU_ONLY
LIBS=-pthread
else
LIBS=-lpthread
endif

CFLAGS += -DUSER_SPACE

#EX_xxxx is set by the user
ALL_CFLAGS=$(CFLAGS) $(EX_CFLAGS)
ALL_LIBS=$(LIBS) $(EX_LIBS)
ALL_LIB_DIR=$(LIB_DIR) $(EX_LIB_DIR)

LDFLAGS=$(ALL_LIB_DIR) $(ALL_LIBS)

#get all .o from the current dir
SOURCE  = $(wildcard *.c)
OBJS = $(patsubst %.c,%.o,$(SOURCE))

#get rid of the object we do not need
ifneq ($(strip $(DEL_OBJECTS)),)
	OBJECTS=$(addprefix $(buildir)/, $(filter-out $(DEL_OBJECTS),$(OBJS)))
else
	OBJECTS=$(addprefix $(buildir)/, $(OBJS))
endif

$(buildir)/%.o:%.c
	mkdir -p $(buildir)
	gcc -o $@ -c $^ $(ALL_CFLAGS)	

#EX_TARGET is the name for our target
TARGET=$(buildir)/$(EX_TARGET)

MAKER=gcc -o $@ $^ $(LDFLAGS)
#if we want to make a lib
ifneq ($(filter lib%.a, $(EX_TARGET)), )
	MAKER=ar r $@ $^
endif

$(TARGET):$(OBJECTS)
	$(MAKER)

install:
	cp $(TARGET) $(INTEG_BUILD_OUTPUT)

clean:
	rm -rf *.o $(buildir)
