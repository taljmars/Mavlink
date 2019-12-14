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
//@Component
public class ParameterDetailsParser {

    private final org.slf4j.Logger LOGGER = LoggerFactory.getLogger(ParameterDetailsParser.class);

    private static final String ParametersListFileName = "MavlinkParamsCopter";
    private static final String ParametersListSeperator = "\\|\\|";
    private static final Integer ParametersListTokens = 5;

//    private Map<String,ParameterDetail> parametersDetails;

//    @PostConstruct
//    public void init() {
//        parametersDetails = buildParser();
//    }

    public Map<String,ParameterDetail> parse(String filePath) {

        Map<String,ParameterDetail> parametersDetails = new HashMap<String, ParameterDetail>();

        try {
            ClassPathResource resource = new ClassPathResource(filePath);
            File file = resource.getFile();
            if (file == null || !file.exists()) {
                LOGGER.error("Failed to find file '{}'", file.getAbsoluteFile());
                System.err.println("Failed to find file '" + file.getAbsoluteFile() + "'");
                return parametersDetails;
            }

            int i = 0;
            Scanner scanner = new Scanner(file);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
//                System.out.println("Resolving: " + line);
                String[] tokens = line.split(ParametersListSeperator);
                if (tokens.length != ParametersListTokens) { // + 1 because of the last empty section
                    System.err.println("Skipping Line: " + line);
                    continue;
                }

                String paramName = tokens[0];
                String paramUnit = tokens[2];
                String paramRange = tokens[3];
                String paramDescription = tokens[4];

                ParameterDetail parameterDetail = new ParameterDetail(i++, paramName, paramUnit, paramRange, paramDescription);
//                System.out.println(parameterDetail);
                parametersDetails.put(paramName, parameterDetail);
            }
            scanner.close();
        }
        catch (Exception e) {
            LOGGER.error("Failed buildParser configuration file", e);
            System.err.println("Failed buildParser configuration file '" + e.getMessage() + "'");
        }

        return parametersDetails;
    }

//    public ParameterDetail get(msg_param_value m_value) {
//        if (!parametersDetails.containsKey(m_value.getParam_Id()))
//            return null;
//
//        return parametersDetails.get(m_value.getParam_Id());
//    }
}
