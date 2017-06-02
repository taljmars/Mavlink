package com.dronegcs.tester;

import com.generic_tools.environment.Environment;
import com.generic_tools.logger.Logger;
import com.generic_tools.validations.RuntimeValidator;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.ApplicationContext;
import org.springframework.context.annotation.AnnotationConfigApplicationContext;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;


@Configuration
public class AppConfig
{
	public static ApplicationContext context = new AnnotationConfigApplicationContext(AppConfig.class);

    @Bean
    public Environment environment() {
        return new Environment();
    }

    @Bean
    public Logger logger(@Autowired Environment environment) {
        return new Logger(environment);
    }

	@Bean
    public RuntimeValidator runtimeValidator() {
        return new RuntimeValidator();
    }

}
