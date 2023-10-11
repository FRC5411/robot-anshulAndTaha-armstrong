package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 *
 *
 * <h1>PilotProfile</h1>
 *
 * <p>Represents a robot driver's personalized preferences and keybinds that can be used to control
 * the robot to their desires.
 *
 * @author Cody Washington (@Jelatinone)
 */
public final class PilotProfile {

  private final Map<String, Supplier<Object>> PREFERENCES = new HashMap<>();
  private final Map<String, Trigger> KEYBINDINGS = new HashMap<>();
  private final String PILOT_NAME;

  /**
   * Constructor.
   *
   * @param Name Pilot's name to reference against other pilot profiles
   */
  public PilotProfile(final String Name) {
    PILOT_NAME = Name;
  }

  /**
   * Adds a new keybind to the pilot's keybind mapping
   *
   * @param KeybindName Name of keybind
   * @param KeybindRetriever Keybind Trigger
   */
  public PilotProfile addKeybind(final String KeybindName, final Trigger KeybindRetriever) {
    KEYBINDINGS.put(KeybindName, KeybindRetriever);
    return this;
  }

  /**
   * Add a new preference to the pilot's preference mapping
   *
   * @param ValueName Name of preference
   * @param Value Value of preference, can be any object
   */
  public PilotProfile addPreference(final String ValueName, final Supplier<Object> Value) {
    PREFERENCES.put(ValueName, Value);
    return this;
  }

  /**
   * Retrieves a keybind from the pilot's keybind mapping
   *
   * @param KeybindName Name of keybind
   * @return A trigger from the pilot's keybind map
   */
  public Trigger getKeybind(final String KeybindName) {
    return KEYBINDINGS.get(KeybindName);
  }

  /**
   * Retreives a preference from the pilot's preference mapping
   *
   * @param ValueName Name of preference
   * @return An object from the pilot's keybind map
   */
  public Object getPreference(final String ValueName) {
    return PREFERENCES.get(ValueName).get();
  }

  /**
   * Retrieves the pilot's name
   *
   * @return Name of pilot
   */
  public String getName() {
    return PILOT_NAME;
  }
}
