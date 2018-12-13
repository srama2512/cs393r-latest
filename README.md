# 1D ASAMI Implementation README

The behavior for 1D ASAMI can be found in https://github.com/srama2512/cs393r-latest/blob/asami_project/core/python/behaviors/sample_1d.py

Running the behavior will output the following files:
- `smLogger.txt`
- `amLogger.txt`
- `gtSensor.txt`
- `gtActuator.txt` (not for simulation)

The models are evaluated using https://github.com/srama2512/cs393r-latest/blob/asami_project/core/python/behaviors/evaluate1d.py . The syntax for evaluation is
```
python evaluate1d.py --load_dir <path to directory with *.txt files> --save_dir <path to save plots> --simFlag <True / False - using simulator?>
```
