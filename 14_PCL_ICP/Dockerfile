FROM argnctu/ros-caffe-ssd

WORKDIR /opt/
RUN git clone https://github.com/mgualti/PointCloudsPython && \
    cd PointCloudsPython && \
    mkdir build && cd build && \
    cmake .. && make && \
    echo "export PYTHONPATH=/opt/PointCloudsPython:$PYTHONPATH" >> /root/.bashrc 

RUN cd /opt/PointCloudsPython && \
    chmod u+x test.py && \
    ./test.py
