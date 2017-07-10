/****************************************************************************
 *   Copyright (c) 2016 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <dspal_signal.h>
#include <pthread.h>

#include "test_utils.h"
#include "dspal_tester.h"
#include <math.h>


//FIXME: should these be sin or sinf, etc??
#define SQRT  sqrtf
#define SIN   sin
#define COS   cos
#define ACOS  acos
#define ASIN  asin
#define ATAN2 atan2
#define ABS   fabs
#define FABS  fabs


#define SKIP_PTHREAD_KILL
#define DSPAL_TESTER_COND_WAIT_TIMEOUT_IN_SECS 3

class CXXTest
{
public:
	CXXTest();
	~CXXTest();

	int doTests();

private:
	int createAndJoin(void *(*helper)(void *), void *test_var);
	int testInit();
	int testCreate();
	int testSelf();
	int testExit();

	pthread_t m_tid;

	int m_init_test[11];
};

static CXXTest test1;

CXXTest::CXXTest()
{
	for (int i = 0; i <= 10; ++i) {
		m_init_test[i] = 10 - i;
	}
}

CXXTest::~CXXTest()
{
}

int CXXTest::testInit()
{
	for (int i = 0; i <= 10; ++i) {
		if (m_init_test[i] != 10 - i) { FAIL("incorrect initialization value"); }
	}

	return TEST_PASS;
}

int CXXTest::doTests()
{
	LOG_INFO("Running CXX tests");

	if (testInit() != TEST_PASS) { FAIL("C++ init test failed"); }

	if (testCreate() != TEST_PASS) { FAIL("pthread_create test failed"); }

	if (testSelf() != TEST_PASS) { FAIL("pthread_self test failed"); }

	if (testExit() != TEST_PASS) { FAIL("pthread_exit test failed"); }

	return TEST_PASS;
}

int CXXTest::createAndJoin(void *(*helper)(void *), void *test_var)
{
	int rv = pthread_create(&m_tid, NULL, helper, test_var);

	if (rv != 0) { FAIL("thread_create returned error"); }

	rv = pthread_join(m_tid, NULL);

	if (rv != 0) { FAIL("thread_join returned error"); }

	return TEST_PASS;
}

static void *createHelper(void *test_value)
{
	int *v = reinterpret_cast<int *>(test_value);
	(*v) = 1;

	return NULL;
}

int CXXTest::testCreate(void)
{
	int test_value = 0;

	int rv = createAndJoin(createHelper, &test_value);

	if (rv == TEST_PASS && test_value != 1) { FAIL("test value did not change"); }

	return rv;
}

static void *selfHelper(void *thread_self)
{
	pthread_t *v = (pthread_t *)thread_self;
	(*v) = pthread_self();

	return NULL;
}

int CXXTest::testSelf(void)
{
	pthread_t thread_self;

	int rv = createAndJoin(selfHelper, &thread_self);

	if (rv == TEST_PASS && thread_self != m_tid) { FAIL("pthread_self did not return the expected value"); }

	return rv;
}

static void *exitHelper(void *test_value)
{
	pthread_exit(NULL);

	int *v = (int *)test_value;
	(*v) = 1;

	return NULL;
}

int CXXTest::testExit(void)
{
	int test_value = 0;

	int rv = createAndJoin(exitHelper, &test_value);

	if (rv == TEST_PASS && test_value != 0) { FAIL("test value should not have changed"); }

	return rv;
}

// Implementation of DSP side of IDL interface spec
int dspal_tester_test_cxx_static()
{
	int rv = test1.doTests();

	if (rv != TEST_PASS) { FAIL("static initialized CXXTest failed"); }

	return TEST_PASS;
}

int dspal_tester_test_cxx_heap()
{
	CXXTest *test2 = new CXXTest;

	int rv = test2->doTests();

	delete test2;

	if (rv != TEST_PASS) { FAIL("heap allocated CXXTest failed"); }

	return rv;
}
// Test QDSP malloc heap size, up to 2MB
int dspal_tester_test_malloc_()
{
    int rv = TEST_PASS;
    uint32_t i = 0, chunk_size = 10*1024, max_size = 2*1024*1024, size = 0; //max is 2MB;
    uint8_t ** ptr_array = (uint8_t **) malloc(sizeof (uint8_t *) * (max_size/chunk_size));
    for (int i = 0; i < (max_size/chunk_size); i ++)
        ptr_array[i] = NULL;
    for (size = chunk_size, i = 0; size  <= max_size; i ++)   // SLPI default is 300KB
    {
        uint8_t * ptr = (uint8_t *) malloc(chunk_size);
        if (ptr == NULL)
        {
            LOG_ERR("malloc failed with size %dKB", (size+chunk_size)/1024);
            rv = TEST_FAIL;
            break;
        }
        ptr_array[i] = ptr;
        size += chunk_size;
        LOG_ERR("malloc success with size %dKB", size/1024);
    }
    for (int i = 0; i < (max_size/chunk_size); i ++)
        if (ptr_array[i] != NULL) free(ptr_array[i]);
    free(ptr_array);
    return size;
}

//uint32_t aa = 0x3F512255, bb = 0xBF5622D3;
uint32_t aa = 0x3E6C07A1, bb = 0xBF607F8D;

float a1 = *(float *) &aa, b1 = *(float *)&bb;
float func3(float a, float b)
{
    int * p = (int *)0x0;
    float c = atan2f(a1, b1);
    c = atan2f(b1, a1);
    c = atan2(a1, b1);
    c = atan2(b1, a1);
    //*p = 0x1;
    return c;
}
void func2(int * para)
{
    char a[] = "abcdefg";
    int local = * para;
    LOG_ERR("to crash it");
    float b1 = 1e-8, b2 = 2.0, b3 = 0;
    float c = atan2f(b1, b2) + atan2f(b2, b1) + atan2f(b2, b3) + atan2f(b3, b2) + func3(1e-6, 1e-6);
    LOG_ERR("test done it");
    int x=0;
    int y=10/x;
    LOG_ERR("test done again? %d", y);

}
int func1(__uint8_t * para1, int para2)
{
    char b[] = "zyxwvut";
    __uint8_t val = * para1;
    LOG_ERR("to call func");
    func2(&para2);
    return 0;
}
int dspal_tester_test_malloc_2()
{
    char c[] = "hijklmn";
    int size = 1;
    __uint8_t para1 = 2;
    int para2 = 3;
    func1(&para1, para2);
    return size;
}

int dspal_tester_test_malloc()
{
    unsigned int x, y;
    while(1) {
        x = 0xBEB4D826; y = 0x3F69B34B;
        //x = 0x3E6C07A1; y = 0xBF607F8D;

        atan2f((float)x, (float)y);
        atan2f((float)y, (float)x);

        float x2 = *(float*)&x;
        float y2 = *(float*)&y;
        atan2f(x2, y2);
        atan2f(y2, x2);
        LOG_INFO("atan2f result %f, %f, %f, %f, %f", atan2f((float)y, (float)x), x2, y2, (float)x, (float)y);
    }
    return SUCCESS;
}

