import pylab as plt
import matplotlib.dates as mdates
from .gpstime import GPSTime

class StateFieldPlotter(object):
    """
    Plotting utility for processing data, both live during the simulation and in
    post-processing.
    """

    def __init__(self):
        # Clear plot and set plotting ticker parameters
        plt.clf()
        date_locator = mdates.AutoDateLocator()
        plt.gca().xaxis.set_major_formatter(
            mdates.AutoDateFormatter(date_locator))
        plt.gca().xaxis.set_major_locator(date_locator)

    def add_timeseries(self, field, field_data):
        """
        Add time series for a state field named "field" to the plot.
        Expects an array of time-value pairs. The times should be datetime strings.
        """

        # Process times in data
        data_t = [mdates.datestr2num(datapoint[0]) for datapoint in field_data]

        # Process values in data
        if field_data[0][1].count(",") == 2:
            # It's a GPS time
            data_vals = [
                GPSTime(datapoint[1]).to_ns() for datapoint in field_data
            ]
            plt.plot(data_t, data_vals, label=field)
        elif field_data[0][1].count(",") == 3:
            # It's a vector
            data_vals = [datapoint[1].split(",") for datapoint in field_data]
            data_vals_x = [float(dataval[0]) for dataval in data_vals]
            data_vals_y = [float(dataval[1]) for dataval in data_vals]
            data_vals_z = [float(dataval[2]) for dataval in data_vals]
            plt.plot(data_t, data_vals_x, label=field + ".x")
            plt.plot(data_t, data_vals_y, label=field + ".y")
            plt.plot(data_t, data_vals_z, label=field + ".z")
        elif field_data[0][1].count(",") == 4:
            # It's a quaternion
            data_vals = [datapoint[1].split(",") for datapoint in field_data]
            data_vals_w = [float(dataval[0]) for dataval in data_vals]
            data_vals_x = [float(dataval[1]) for dataval in data_vals]
            data_vals_y = [float(dataval[2]) for dataval in data_vals]
            data_vals_z = [float(dataval[3]) for dataval in data_vals]
            plt.plot(data_t, data_vals_w, label=field + ".w")
            plt.plot(data_t, data_vals_x, label=field + ".x")
            plt.plot(data_t, data_vals_y, label=field + ".y")
            plt.plot(data_t, data_vals_z, label=field + ".z")
        else:
            if field_data[0][1] in ["true", "false"]:
                # It's a boolean
                data_vals = [(1 if datapoint == "true" else 0)
                             for datapoint in field_data]
            else:
                try:
                    # It might be an integer
                    data_vals = [int(datapoint[1]) for datapoint in field_data]
                except ValueError:
                    try:
                        # It's a float or double
                        data_vals = [
                            float(datapoint[1]) for datapoint in field_data
                        ]
                    except ValueError:
                        print(f"Field {field} is not of a plottable type.")
                        return

            plt.plot(data_t, data_vals, label=field)

        return True

    def display(self):
        plt.gcf().autofmt_xdate()
        plt.legend()
        plt.show()
