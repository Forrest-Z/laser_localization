#!/bin/sh

# docbookoutput. The when this test fails due to e.g. formatting
# changes the results needs to be manually reviewed and the test81.out
# updated
./test_wrapper $srcdir/test84.out ../examples/test25 '-h x'
