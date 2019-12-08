# GROUND SOFTWARE
Ground Software will store/index all telemetry coming from the Iridium email account 
using a Flask server connected to ElasticSearch. Telemetry will be stored in two 
different indexes in ElasticSearch

1) Statefield_Reports: Statefield reports contain updated statefields, values, and times
2) Iridium_Reports: Iridium reports include the most recent MOMSN and MTMSN (from both downlinks
and uplink confirmations) numbers. It also includes whether or not the ground is able to send 
uplinks and the time of the report.

The Flask server contains two endpoints for accessing information from either index in ElasticSearch.
These endpoints are used in RadioSession to check whether or not it can send uplinks and for
reading the most updated statefield variables.

# Installing

ElasticSearch:
1. If you are using a Macbook, you can can install ElasticSearch