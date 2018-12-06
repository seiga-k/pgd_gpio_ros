# gpio message
Header header           # timestamp.
int32[] port_numbers    # port numbers of the elements.
int32[] masks           # data will ignore if set one for the corresponding element.
int32[] datas           # gpio port status. Port logic sign are defined by parameter.