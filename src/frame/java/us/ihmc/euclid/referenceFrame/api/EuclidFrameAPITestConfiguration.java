package us.ihmc.euclid.referenceFrame.api;

/**
 * Interface that can be implemented to define a configuration for the API tester and its builder.
 * <p>
 * This interface is part of the API testing framework.
 * </p>
 *
 * @see EuclidFrameAPITester
 * @author Sylvain Bertrand
 */
public interface EuclidFrameAPITestConfiguration
{
   /**
    * Configure the API tester and its builder.
    * 
    * @param testerToConfigure  the API tester to configure. This includes registering frame types,
    *                           frameless types, and exception types to ignore during testing.
    * @param builderToConfigure the builder to configure. This includes registering random frame type
    *                           and frameless type generators.
    */
   void configure(EuclidFrameAPITester testerToConfigure, ReflectionBasedBuilder builderToConfigure);
}
