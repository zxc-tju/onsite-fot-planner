FROM huangyan520/onsite:1.0.4

COPY ./planner ./planner

COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple/

COPY ./frenet_optimal_trajectory_planner ./frenet_optimal_trajectory_planner

RUN apt-get upgrade -y

RUN apt-get update && apt-get install qtbase5-dev -y qtchooser qt5-qmake qtbase5-dev-tools

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y libeigen3-dev clang cmake

WORKDIR frenet_optimal_trajectory_planner

RUN bash build.sh

WORKDIR ..

ENTRYPOINT python /planner
