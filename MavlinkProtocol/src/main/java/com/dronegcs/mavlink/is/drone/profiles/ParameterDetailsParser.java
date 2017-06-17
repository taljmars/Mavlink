package com.dronegcs.mavlink.is.drone.profiles;

import com.dronegcs.mavlink.is.protocol.msg_metadata.ardupilotmega.msg_param_value;
import org.slf4j.LoggerFactory;
import org.springframework.core.io.ClassPathResource;
import org.springframework.stereotype.Component;

import javax.annotation.PostConstruct;
import java.io.File;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

/**
 * Created by taljmars on 6/17/2017.
 */
@Component
public class ParameterDetailsParser {

    private final org.slf4j.Logger LOGGER = LoggerFactory.getLogger(ParameterDetailsParser.class);

    private static final String ParametersListFileName = "MavlinkParams";
    private static final String ParametersListSeperator = "\\|\\|";
    private static final Integer ParametersListTokens = 7;

    Map<String,String> parametersDetails;

    @PostConstruct
    public void init() {
        parametersDetails = parse();
    }

    private Map<String, String> parse() {

        Map res = new HashMap<String, String>();

        try {
            ClassPathResource resource = new ClassPathResource("/com/dronegcs/mavlink/" + ParametersListFileName);
            InputStream inputStream = resource.getInputStream();
            File file = resource.getFile();
            if (file == null || !file.exists()) {
                LOGGER.error("Failed to find file '{}'", file.getAbsoluteFile());
                return res;
            }

            Scanner scanner = new Scanner(file);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] tokens = line.split(ParametersListSeperator);
                if (tokens.length != ParametersListTokens + 1) // + 1 because of the last empty section
                    continue;

                String paramName = tokens[1];
                String paramDescription = tokens[7];

                res.put(paramName, paramDescription);
            }
            scanner.close();
        }
        catch (Exception e) {
            LOGGER.error("Failed parse configuration file", e);
        }

        return res;
    }

    public String get(msg_param_value m_value) {
        if (!parametersDetails.containsKey(m_value.getParam_Id()))
            return "Not exist";

        return parametersDetails.get(m_value.getParam_Id());
    }
}
