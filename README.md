# gh-mavros
<h2>Install in a new workspace: </h2>
<ol>
<li>cd ~</li>
<li>mkdir new_ws</li>
<li>cd new_ws</li>
<li>mkdir src</li>
<li>catkin_init_workspace</li>
<li>git clone https://github.com/art-mx/gh-mavros.git</li>
<li>cd ~/new_ws</li>
<li>catkin_make</li>
<li>echo "source ~/new_ws/devel/setup.bash" >> ~/.bashrc</li>
<li>source ~/.bashrc</li>
</ol>


<h2>Run:</h2>
rosrun gh-mavros gh_commander.py


