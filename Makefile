
all: 
	@echo "Use doc"

doc:
	@echo "Running doxygen..."
	@rm -rf doc
	@mkdir doc
	doxygen doxygen.conf
	@echo "done."
	
	
