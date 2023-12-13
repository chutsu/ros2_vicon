help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
		{printf "\033[1;36m%-12s\033[0m%s\n", $$1, $$2}'

vicon_datastream_sdk:
	@echo "Downloading Vicon DataStream SDK 1.12"
	@wget -q --show-progress https://app.box.com/shared/static/7s2j1j2oec1d19up2p5se5tlqrsky149.zip
	@unzip 7s2j1j2oec1d19up2p5se5tlqrsky149.zip
	@mv 20230413_145507h vicon_datastream_sdk
	@rm 7s2j1j2oec1d19up2p5se5tlqrsky149.zip

deps: vicon_datastream_sdk  ## Install Vicon DataStream SDK
