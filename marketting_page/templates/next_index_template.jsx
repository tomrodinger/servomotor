import Head from 'next/head';
import Image from 'next/image';
import Link from 'next/link';
import styles from '../styles/Marketing.module.css';

export default function Home() {
  return (
    <>
      <Head>
        <title>M17 Series Servomotors - Gearotons</title>
        <meta name="description" content="Affordable and Simple All-in-One Motion Control - M17 Series Servomotors from Gearotons" />
        <link rel="icon" href="/favicon.ico" />
        <style jsx global>{`
          body {
            background-color: #ffffff !important;
            color: #333 !important;
          }
          main {
            background-color: #ffffff !important;
            color: #333 !important;
            padding: 0 !important;
            min-height: auto !important;
            display: block !important;
            justify-content: initial !important;
            align-items: initial !important;
          }
        `}</style>
      </Head>
      <div className={styles.marketing}>

      <main>
        {/* Hero Section */}
        <header className={styles.heroSection}>
          <div className={styles.container}>
            <Image
              src="/marketing/logos/Gearotons_Logo.png"
              alt="Gearotons Logo"
              className={styles.logo}
              width={__WIDTH_Gearotons_Logo__}
              height={__HEIGHT_Gearotons_Logo__}
            />
            <h1 className={styles.mainTitle}>M17 Series Servomotors</h1>
            <p className={styles.subtitle}>Affordable and Simple All-in-One Motion Control</p>
            <p className={styles.subtitle}>From Education to Innovation</p>
            <Image
              src="/marketing/images/M17_series_overview.jpg"
              alt="M17 Series Overview"
              className={styles.heroImage}
              width={__WIDTH_M17_series_overview__}
              height={__HEIGHT_M17_series_overview__}
            />
          </div>
        </header>

        {/* Shop Now CTA */}
        <div className={styles.ctaContainer}>
          <Link href="/store" className={styles.ctaButton}>
            Shop Now
          </Link>
        </div>

        {/* Introduction Section */}
        <section id="introduction" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Introduction</h2>
            __INTRO_PARAGRAPHS__
            <div className={styles.sectionImageContainer}>
              <Image
                src="/marketing/images/one_motor_small.jpg"
                alt="M17 Servomotor"
                className={styles.sectionImage}
                width={__WIDTH_one_motor_small__}
                height={__HEIGHT_one_motor_small__}
              />
            </div>
          </div>
        </section>

        {/* Features Section */}
        <section id="features" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Key Features</h2>
            <div className={styles.sectionImageContainer}>
              <Image
                src="/marketing/images/kit_with_three_motors_small.jpg"
                alt="M17 Servomotor Kit"
                className={styles.sectionImage}
                width={__WIDTH_kit_with_three_motors_small__}
                height={__HEIGHT_kit_with_three_motors_small__}
              />
            </div>
            <ul className={styles.featuresListSimple}>
            __FEATURES_LI__
            </ul>
          </div>
        </section>

        {/* Connection Diagram */}
        <section id="connection" className={`${styles.contentSection} ${styles.diagramSection}`}>
          <div className={styles.container}>
            <h2>Connection Diagram</h2>
          </div>
          <div className={styles.diagramContainer}>
            <Image
              src="/marketing/images/connection_diagram.jpg"
              alt="Connection Diagram"
              className={styles.diagramImage}
              width={__WIDTH_connection_diagram__}
              height={__HEIGHT_connection_diagram__}
            />
          </div>
        </section>

        {/* Unit System */}
        <section id="unit-system" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Unit System</h2>
            <p>The M17 Series Servomotors have certain internal units so that they can perform the calculations associated with motion efficiently (using integer math). It is the responsibility of the controlling software to support multiple units of measurement for various quantities. Our Python and Arduino libraries handle unit conversions automatically, allowing you to work with your preferred units. Below are the supported units for each quantity:</p>
            __UNIT_TABLE__
          </div>
        </section>

        {/* Getting Started Guide */}
        <section id="getting-started" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Getting Started Guide</h2>
            <p>To help you get started with your M17 Series Servomotor, we provide a comprehensive online guide that covers everything from initial setup to advanced protocol implementations. This guide includes:</p>
            <ul className={styles.guideFeatures}>
              <li>Step-by-step setup instructions</li>
              <li>Detailed communication protocol documentation</li>
              <li>Programming examples and code snippets</li>
              <li>Description of error codes</li>
              <li>Troubleshooting tips and best practices</li>
            </ul>
            <div className={styles.guideLink}>
              <Image
                src="/marketing/images/click_here.png"
                alt="Click Here"
                className={styles.clickIcon}
                width={__WIDTH_click_here__}
                height={__HEIGHT_click_here__}
              />
              <a href="https://servo-tutorial.netlify.app/" target="_blank" className={`${styles.btn} ${styles.btnPrimary}`}>
                Click Here to Visit our Getting Started Guide
              </a>
            </div>
          </div>
        </section>

        {/* Indicators Section */}
        <section id="indicators" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Indicator LEDs and Buttons</h2>
            <div className={styles.sectionImageContainer}>
              <Image
                src="/marketing/images/motor_back_small.jpg"
                alt="Motor Back with LEDs and Buttons"
                className={styles.sectionImage}
                width={__WIDTH_motor_back_small__}
                height={__HEIGHT_motor_back_small__}
              />
            </div>
            <p>The servomotor has two status LEDs (Green and Red). The green LED flashes slowly to show a heart beat and quickly to indicate that the bootloader is running rather than the application. The red LED will light up briefly to show communication on the bus and will indicate fatal error codes by flashing a certain number of times.</p>
            <p>The servomotor has two buttons labelled "Reset" and "Test". The Reset button will reset the internal microcontroller and all state will go back to default values. The Test button will cause the motor to spin. Press briefly to let it spin one way and press for more than 0.3 seconds and release to let it spin the other way. Hold down for at least 2 seconds and release to cause the motor to go to closed loop mode. Hold down for more than 15 seconds and release to let the motor perform a calibration on itself. Note that it will spin during calibration and must be able to spin freely for calibration to be successful, so remove any loads before doing this operation.</p>
          </div>
        </section>

        {/* Communication Protocol */}
        <section id="protocol" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Communication Protocol</h2>
            <div className={styles.sectionImageContainer}>
              <Image
                src="/marketing/images/adapter_and_wire_small.jpg"
                alt="RS-485 Adapter and Wire"
                className={styles.sectionImage}
                width={__WIDTH_adapter_and_wire_small__}
                height={__HEIGHT_adapter_and_wire_small__}
              />
            </div>
            <p>The M17 series uses RS-485 communication with a simple command-based protocol. Multiple motors can be daisy-chained on a single bus, each with a unique ID.</p>
            <h3>Command Reference Summary</h3>
            <p>For the up to date source of truth for all available commands, you can look at this document.</p>
            <div className={styles.commandLink}>
              <Image
                src="/marketing/images/click_here.png"
                alt="Click Here"
                className={styles.clickIconSmall}
                width={__WIDTH_click_here__}
                height={__HEIGHT_click_here__}
              />
              <a href="https://github.com/tomrodinger/servomotor/blob/main/python_programs/servomotor/motor_commands.json" target="_blank">
                https://github.com/tomrodinger/servomotor/blob/main/python_programs/servomotor/motor_commands.json
              </a>
            </div>
            <p>You can also run this command:</p>
            <pre><code className="language-bash">{`pip3 install servomotor   # run this just once to install the library and programs
servomotor_command.py -c`}</code></pre>
            <p>This will print out the information contained in the motor_commands.json file in a nicer way and give some usage information for sending commands to the motor from the command line.</p>
            <p>The commands are grouped by functionality including Basic Control, Motion Control, Configuration, Status & Monitoring, and Device Management.</p>
          </div>
        </section>

        {/* Specifications */}
        <section id="specifications" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Technical Specifications</h2>

            <h3>Mechanical Specifications</h3>
            <table className={styles.specsTable}>
              <thead>
                <tr>
                  <th>Parameter</th>
                  <th>M17-60</th>
                  <th>M17-48</th>
                  <th>M17-40</th>
                </tr>
              </thead>
              <tbody>
                <tr><td>Dimensions (LxW)</td><td>42.2x42.2 mm</td><td>42.2x42.2 mm</td><td>42.2x42.2 mm</td></tr>
                <tr><td>Height</td><td>59.8 mm</td><td>48.6 mm</td><td>41.6 mm</td></tr>
                <tr><td>Shaft Length</td><td>20.4 mm</td><td>20.4 mm</td><td>18.5 mm</td></tr>
                <tr><td>Shaft Diameter</td><td>5 mm</td><td>5 mm</td><td>5 mm</td></tr>
                <tr><td>Weight</td><td>470g</td><td>360g</td><td>285g</td></tr>
                <tr><td>Protection Class</td><td>IP20</td><td>IP20</td><td>IP20</td></tr>
              </tbody>
            </table>

            <div className={styles.dimensionImages}>
              <div className={styles.dimensionImageContainer}>
                <Image
                  src="/marketing/images/M17-60_dimensions.png"
                  alt="M17-60 Dimensions"
                  className={styles.dimensionImage}
                  width={__WIDTH_M17_60_dimensions__}
                  height={__HEIGHT_M17_60_dimensions__}
                />
                <p className={styles.dimensionLabel}>M17-60</p>
              </div>
              <div className={styles.dimensionImageContainer}>
                <Image
                  src="/marketing/images/M17-48_dimensions.png"
                  alt="M17-48 Dimensions"
                  className={styles.dimensionImage}
                  width={__WIDTH_M17_48_dimensions__}
                  height={__HEIGHT_M17_48_dimensions__}
                />
                <p className={styles.dimensionLabel}>M17-48</p>
              </div>
              <div className={styles.dimensionImageContainer}>
                <Image
                  src="/marketing/images/M17-40_dimensions.png"
                  alt="M17-40 Dimensions"
                  className={styles.dimensionImage}
                  width={__WIDTH_M17_40_dimensions__}
                  height={__HEIGHT_M17_40_dimensions__}
                />
                <p className={styles.dimensionLabel}>M17-40</p>
              </div>
            </div>

            <h3>Electrical Specifications</h3>
            <table className={styles.specsTable}>
              <thead>
                <tr>
                  <th>Parameter</th>
                  <th>M17-60</th>
                  <th>M17-48</th>
                  <th>M17-40</th>
                </tr>
              </thead>
              <tbody>
                <tr><td>Operating Voltage</td><td>12-24V</td><td>12-24V</td><td>12-24V</td></tr>
                <tr><td>Rated Torque</td><td>0.65 N.m</td><td>0.55 N.m</td><td>0.42 N.m</td></tr>
                <tr><td>Maximum Speed</td><td>560 RPM</td><td>560 RPM</td><td>560 RPM</td></tr>
                <tr><td>Maximum Current</td><td>1.1A</td><td>1.1A</td><td>1.1A</td></tr>
                <tr><td>Rated Power</td><td>38W</td><td>32W</td><td>25W</td></tr>
              </tbody>
            </table>

            <h3>Operating Conditions</h3>
            <table className={styles.conditionsTable}>
              <thead>
                <tr>
                  <th>Parameter</th>
                  <th>Specification</th>
                </tr>
              </thead>
              <tbody>
                <tr><td>Operating Temperature</td><td>0°C to +80°C</td></tr>
                <tr><td>Storage Temperature</td><td>-20°C to +60°C</td></tr>
                <tr><td>Humidity Range</td><td>20% to 80% RH (non-condensing)</td></tr>
                <tr><td>Installation Environment</td><td>Indoor use only</td></tr>
              </tbody>
            </table>
          </div>
        </section>

        {/* Library Support */}
        <section id="libraries" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Library Support</h2>

            <div className={styles.librarySection}>
              <h3>Python Library</h3>
              <p>Easy-to-use Python library for controlling M17 servomotors:</p>
              <pre><code className="language-python">{`__PYTHON_EXAMPLE__`}</code></pre>
            </div>

            <div className={styles.librarySection}>
              <h3>Arduino Library</h3>
              <p>Arduino library for easy integration with Arduino boards:</p>
              <pre><code className="language-cpp">{`__ARDUINO_EXAMPLE__`}</code></pre>
            </div>
          </div>
        </section>

        {/* Applications */}
        <section id="applications" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Applications</h2>
            <p>The M17 Series servomotors are perfect for a wide range of applications, from educational projects to industrial automation.</p>
            <div className={styles.applicationsGrid}>
              <div className={styles.applicationCard}>
                <Image
                  src="/marketing/images/robotics_small.jpg"
                  alt="Robotics Application"
                  className={styles.applicationImage}
                  width={__WIDTH_robotics_small__}
                  height={__HEIGHT_robotics_small__}
                />
                <h3>Robotics</h3>
                <p>Build precise robotic arms, mobile robots, and educational robotics platforms with easy-to-control servomotors.</p>
              </div>
              <div className={styles.applicationCard}>
                <Image
                  src="/marketing/images/automation_small.jpg"
                  alt="Automation Application"
                  className={styles.applicationImage}
                  width={__WIDTH_automation_small__}
                  height={__HEIGHT_automation_small__}
                />
                <h3>Automation</h3>
                <p>Perfect for automated systems, CNC machines, 3D printers, and industrial control applications.</p>
              </div>
            </div>
          </div>
        </section>

        {/* Company Profile */}
        <section id="company-profile" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Company Profile</h2>
            <div className={styles.companyContent}>
            __COMPANY_PARAGRAPHS__
              <div className={styles.companyImageContainer}>
                <Image
                  src="/marketing/images/test_rack_small.jpg"
                  alt="Test Rack"
                  className={styles.companyImage}
                  width={__WIDTH_test_rack_small__}
                  height={__HEIGHT_test_rack_small__}
                />
                <p className={styles.imageCaption}>Our testing facility ensures every motor meets quality standards</p>
              </div>
            </div>
          </div>
        </section>

        {/* Open Source */}
        <section id="open-source" className={styles.contentSection}>
          <div className={styles.container}>
            <h2>Open Source</h2>
            <div className={styles.openSourceContent}>
              <p>We believe in making the world better through technology. All software, firmware, and PCB design files are available here:</p>
              <div className={styles.githubLink}>
                <Image
                  src="/marketing/images/click_here.png"
                  alt="Click Here"
                  className={styles.clickIconSmall}
                  width={__WIDTH_click_here__}
                  height={__HEIGHT_click_here__}
                />
                <a href="https://github.com/tomrodinger/servomotor" target="_blank">
                  https://github.com/tomrodinger/servomotor
                </a>
              </div>
              <div className={styles.openSourceLogos}>
                <Image
                  src="/marketing/images/Open-source-hardware-logo.svg.png"
                  alt="Open Source Hardware"
                  className={styles.oshLogo}
                  width={__WIDTH_Open_source_hardware_logo_svg__}
                  height={__HEIGHT_Open_source_hardware_logo_svg__}
                />
                <Image
                  src="/marketing/images/Open_Source_Initiative.svg.png"
                  alt="Open Source Initiative"
                  className={styles.osiLogo}
                  width={__WIDTH_Open_Source_Initiative_svg__}
                  height={__HEIGHT_Open_Source_Initiative_svg__}
                />
              </div>
            </div>
          </div>
        </section>

        {/* Footer */}
        <footer className={styles.footer}>
          <div className={styles.container}>
            <Image
              src="/marketing/logos/Gearotons_Logo.png"
              alt="Gearotons"
              className={styles.footerLogo}
              width={__WIDTH_Gearotons_Logo__}
              height={__HEIGHT_Gearotons_Logo__}
            />
            <p className={styles.versionInfo}>Version __VERSION__ - __DATE__</p>
            <p className={styles.copyright}>© 2024 Gearotons. All specifications subject to change without notice.</p>
            <p>For more information and technical support, please contact our sales team.</p>
          </div>
        </footer>
      </main>

      {/* Add syntax highlighting scripts */}
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/prism.min.js"></script>
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/components/prism-python.min.js"></script>
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/1.29.0/components/prism-cpp.min.js"></script>
      </div>
    </>
  );
}