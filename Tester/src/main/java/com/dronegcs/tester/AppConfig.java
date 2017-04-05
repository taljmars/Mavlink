package com.dronegcs.tester;

import org.springframework.context.ApplicationContext;
import org.springframework.context.annotation.AnnotationConfigApplicationContext;
import org.springframework.context.annotation.Configuration;


@Configuration
public class AppConfig
{

	public static ApplicationContext context = new AnnotationConfigApplicationContext(AppConfig.class);

}
