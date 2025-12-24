# will overwrite the robot.env file with whatever text is inside ROBOT_ENV_VAR

if [ -n "${ROBOT_ENV_VAR}" ]; then
    mv robot.env tmp_robot.env
    echo "${ROBOT_ENV_VAR}" > robot.env
fi
