# Frenet Optimal Planner for Onsite

frenet optimal planner can be installed with::

	git clone https://github.com/ZhaoXiaocong20/frenet_optimal_trajectory_planner.git


## docker upload process
sudo docker login

sudo docker build -t id/name:version .

sudo docker push id/name:version

## local test
sudo docker run -v /home/desmond/onsite/inputs:/inputs -v /home/desmond/onsite/outputs:/outputs id/name:version
