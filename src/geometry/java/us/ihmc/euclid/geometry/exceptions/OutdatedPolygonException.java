package us.ihmc.euclid.geometry.exceptions;

/**
 * {@link RuntimeException} dedicated to improper use of a polygon with vertices that are not
 * up-to-date.
 * 
 * @author Sylvain Bertrand
 *
 */
public class OutdatedPolygonException extends RuntimeException
{
   private static final long serialVersionUID = -5043468839061602341L;

   /**
    * Creates a new exception.
    * 
    * @param message the detail message.
    */
   public OutdatedPolygonException(String message)
   {
      super(message);
   }
}