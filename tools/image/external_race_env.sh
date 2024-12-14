# will overwrite the race.env file with whatever text is inside RACE_ENV_VAR

if [ -n "${RACE_ENV_VAR}" ]; then
    mv race.env tmp_race.env
    echo "${RACE_ENV_VAR}" > race.env
fi
