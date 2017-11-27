RSBB general benchmarks configuration
=================================================

An example of the general benchmarks configuration:

```yaml
base_results_directory: "~/rsbb_output/"
base_record_directory: "~/rsbb_output/"
general_record_list:
  - /tf
  - /rosout
  - /bmbox/map
  - /bmbox/map_metadata
  - /bmbox/bmbox_state
  - /bmbox/refbox_state
  - /bmbox/current_result
  - /bmbox/benchmark_configuration
  - /core/to_gui
```



## General parameters

The *general.yaml* configuration file contains the general configuration for all benchmarks.

All of the following parameters must be specified.
`TODO: check` If a parameter has an invalid value or is not specified, the RSBB or the components of the RSBB using the parameter will not start and print an error message specifying the incorrect configuration.


### base_results_directory

This parameter should be a string.

The *base_results_directory* parameter specifies the directory path where the results of the benchmarks are saved.
The directory should exist and be a valid path, meaning that the path should point to a directory or to a link, and not to a file.

This parameter is used by the BmBox.


### base_record_directory

This parameter should be a string.

The *base_record_directory* parameter specifies the directory path where the logs of the benchmarks are saved.
The directory should exist and be a valid path, meaning that the path should point to a directory or to a link, and not to a file.

This parameter is used by the record server.


### general_record_list

:warning: `Note: in non scripted benchmarks the recording is not implemented yet.`

This parameter should be a sequence of strings (see examples in the beginning of this section and the following specification).

The *general_record_list* parameter specifies the topics recorded by the record server during the benchmark execution.
Note that the list of topics specified in the *benchmarks_description.yaml* configuration file as *record_topics* is also recorded for each benchmark.

This parameter is used by the RefBox and by the record server.
