clean:
	find . -type f -name "*.slxc" -delete
	find . -type f -name "*.asv" -delete
	find . -type d -name "generated" -exec sh -c 'cd "{}" && rm -rf ./*' \;
	find . -type d -name "slprj" -exec rm -rf {} +
	rm -rf figures
