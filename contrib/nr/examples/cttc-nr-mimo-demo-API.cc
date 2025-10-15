// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: 2025 Aleksei Fedorov <aleksei.fedorov@eit.lth.se>
// SPDX-FileCopyrightText: 2025 Nikita Lyamin <nikita.lyamin@volvocars.com>

/**
 * \ingroup examples
 * \file cttc-nr-mimo-demo.cc
 * \brief An example that shows how to setup and use MIMO
 *
 * This example describes how to setup a simulation using MIMO. The scenario
 * consists of a simple topology, in which there
 * is only one gNB and one UE. An additional pair of gNB and UE can be enabled
 * to simulate the interference (see enableInterfNode).
 * Example creates one DL flow that goes through only BWP.
 *
 * The example prints on-screen and into the file the end-to-end result of the flow.
 * To see all the input parameters run:
 *
 * \code{.unparsed}
$ ./ns3 run cttc-nr-mimo-demo -- --PrintHelp
    \endcode
 *
 *
 * MIMO is enabled by default. To disable it run:
 *  * \code{.unparsed}
$  ./ns3 run cttc-nr-mimo-demo -- --enableMimoFeedback=0
    \endcode
 *
 *
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include <ns3/sivert-three-gpp-spectrum-propagation-loss-model.h>
#include "ns3/spectrum-value.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-channel.h"
#include "ns3/nr-spectrum-phy.h"
#include "ns3/log.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("CttcNrMimoDemoApi");

#include <thread>
#include <chrono>

// Includes for SIVERT API
#include "flatbuffer_structures/zhelpers.hpp" // For ease we use zhelpers instead of zmq.hpp
#include <zmq.hpp>
#include "flatbuffer_structures/positionSchema_EMI_vectorised_generated.h" // include for our API's flatbuffer structure - channel with EMI
#include "flatbuffer_structures/MsgReceivedSchema_generated.h" // include for our API's flatbuffer structure

#include <random>

using namespace ns3;
using namespace SivertAPI::PosUpd; // namespace for Position API's flatbuffers structures
using namespace SivertAPI::MsgReceived; // namespace for Msg API's flatbuffers structures


/// @brief Global variables for ZeroMQ API connections
zmq::context_t context (1); // NB: there should be only one context in one thread --> thus, SINGLE context for entire NS3 simulation.
// Global variables for Pub and Sub ZeroMQ API connections
zmq::socket_t* APIconnect_sub;
zmq::socket_t* APIconnect_pub;

// Global EMI PSD vector
std::vector<double> EMI_psd;
bool IsEMIdisabled; // defining globally to pass around

int GetRandomIntInRange(int min, int max)
{
    static std::random_device rd;   // seed
    static std::mt19937 gen(rd());  // Mersenne Twister engine
    std::uniform_int_distribution<> distrib(min, max);
    return distrib(gen);
}

double WtodBmlocalContext(double w)
{
  double dbm = std::log10(w * 1000.0) * 10.0;
  return dbm;
}


// Super IMPORTANT! We should use only one ZMQ context for our API!
static zmq::socket_t * s_sub_socket (zmq::context_t & context, std::string address) {

/*
 * For our needs one ZeroMQ context is enough (as a rule of thumb 1 context is required per 1 Gbit/s data exchange).
 * However, one could create numerous sockets using different ZeroMQ patterns for the API needs. Currently we use
 * Cleint/Server socket and PUB/SUB socket for our API.
*/

    zmq::socket_t * sck = new zmq::socket_t (context, ZMQ_SUB);
    // std::unique_ptr<zmq::socket_t> sck(new zmq::socket_t(context, ZMQ_SUB));
    sck->setsockopt( ZMQ_LINGER, 0 );             // ALWAYS, best before .bind()/.connect()
//    sck->setsockopt(ZMQ_RCVBUF, 1000000);
    sck->bind(address);
    sck->setsockopt( ZMQ_SUBSCRIBE, "toNS", 4);
    std::cout << "API SUB socket is created..." << std::endl;

    return sck;
}

// Super IMPORTANT! We should use only one ZMQ context for our API!
static zmq::socket_t * s_pub_socket (zmq::context_t & context, std::string address) {

/*
 * For our needs one ZeroMQ context is enough (as a rule of thumb 1 context is required per 1 Gbit/s data exchange).
 * However, one could create numerous sockets using different ZeroMQ patterns for the API needs. Currently we use
 * Cleint/Server socket and PUB/SUB socket for our API.
*/

    zmq::socket_t * sck = new zmq::socket_t (context, ZMQ_PUB);
    // std::unique_ptr<zmq::socket_t> sck(new zmq::socket_t(context, ZMQ_PUB));
    sck->setsockopt( ZMQ_LINGER, 0 );             // ALWAYS, best before .bind()/.connect()
//    sck->setsockopt(ZMQ_RCVBUF, 1000000);
    sck->bind(address);
//    sck->setsockopt( ZMQ_, "fromNS", 6);
    std::cout << "API PUB socket is created..." << std::endl;

    return sck;
}

// @brief SIVERT API main entry point
void SetAllParametersFromApiWithInterferenceSwitch (NodeContainer gnbNodes, NodeContainer ueNodes, zmq::socket_t * APIconnect, double FixedUpdInterval, const Ptr<SivertThreeGppSpectrumPropagationLossModel> SpectrumModelPtr)
{
/*
 * APIconnect - pointer to ZeroMQ socket SUB socket created in the main
 * FixedUpdInterval - Unity3D FixedUpdInterval we get from intiSocket from Unity.
 * 					  FixedUpd defines the physics updates in Unity and we want to synch NS3
 * 					  via our API  every time new values are calculated in Unity: for this we
 * 					  call PUB API each void FixedUpd and call the API SUB on NS3 side.
 *
*/


    std::cout<< "Receiving update from Unity3D"<<std::endl;
    
    std::cout << "Waiting for topic..." << std::endl;

    std::string topic = s_recv (*APIconnect); // receive TOPIC

    std::cout << "Got topic: " << topic << std::endl;

    std::string msg = s_recv (*APIconnect); // receive API message

    std::cout<< "Unity3D update recived, processing.. "<< topic << std::endl;
    // Unwrapping the Flatbuffer API data structure we received from Unity
    flatbuffers::FlatBufferBuilder fbb;
    PosAPIBuilder builder(fbb);
    auto APIdata = GetPosAPI(msg.data());
    auto info = APIdata->Suppl();
    auto posit = APIdata->pos();
    bool termin = APIdata->terminateNS3();
//  auto EmergencyBrake = APIdata->CITS();
    // auto EmergencyBrake = APIdata->CITS();
    auto GSCMunity = APIdata->GSCM();
    auto GSCMvec = APIdata->GSCMvector();


  auto SpectumChVector = APIdata->GSCMSpectruChannels();
  // auto SpectrumChInfo = APIdata->SpectrumInfo();
  auto NumFr = APIdata->NumberOfFrequnciesPerChannel();
  

    auto spectrumValues = APIdata->GSCMSpectruChannels();
    for (const auto* spectrumValue : *spectrumValues) {
        // std::cout << "ChannelID: " << spectrumValue->ChannelID() << std::endl;
        // std::cout << "PSD Gain Coefficient:  " << spectrumValue->PSDGainCoefficient()->Get(100) << std::endl;
        // std::cout << "TxID: " << spectrumValue->TxID() << std::endl;
        // std::cout << "RxID: " << spectrumValue->RxID() << std::endl;
        // std::cout << "IsEMIChannel: " << spectrumValue->IsEMIChannel() << std::endl;
        // std::cout << "CoefInd: " << spectrumValue->CoefInd() << std::endl;

        // Access the PSDGainCoefficient vector
        auto psdGains = spectrumValue->PSDGainCoefficient();
        std::vector<double> gscmSpectrumGain;
        std::vector<std::complex<double>> gscmSpectrumGain_complex;
        for (size_t i = 0; i < psdGains->size(); i++) {
            // std::cout << "PSD Gain Coefficient[" << i << "]: " << psdGains->Get(i) << std::endl;
            gscmSpectrumGain.push_back(psdGains->Get(i));
        }
        if (!spectrumValue->IsEMIChannel())
        { 
        //   gscmSpectrumGainEmiNormalised = gscmSpectrumGain;   
        //   std::cout << "Setting channel from Unity3D for TxID: " << spectrumValue->TxID()-6 << " RxID: " << spectrumValue->RxID() << std::endl;
        //   std::cout << "Spectrum Gain: " << gscmSpectrumGain.size() << " values." << std::endl;  
          
          int _fft = gscmSpectrumGain.size()/3;
          for (size_t i = 0; i < _fft; i++)
          {
            gscmSpectrumGain_complex.push_back(std::complex<double>(gscmSpectrumGain[i+_fft], gscmSpectrumGain[i+_fft*2]));
            NS_LOG_DEBUG("Complex coef. for RB (from Unity) " << i << ": " << gscmSpectrumGain_complex[i]);
          }
          SpectrumModelPtr->SetChannelFromUnity3D(spectrumValue->TxID(),spectrumValue->RxID(), gscmSpectrumGain);
          SpectrumModelPtr->SetComplexChannelFromUnity3D(spectrumValue->TxID()-2,spectrumValue->RxID(), gscmSpectrumGain_complex);
          NS_LOG_DEBUG("SetChannelFromUnity3D for TxID: " << spectrumValue->TxID()-2 << " RxID: " << spectrumValue->RxID() << " with complex values.");

        }
        else{
          std::cout << "EMI channel detected." << " EMI flag is set in Unity3D to: " << IsEMIdisabled << " EMI ref level: "<< gscmSpectrumGain[50]  << std::endl;
          EMI_psd.clear();
            for (size_t i = 0; i < gscmSpectrumGain.size(); ++i) {
                EMI_psd.push_back(gscmSpectrumGain[i]); // Replace with your actual Unity PSD extraction
            }
          // Here we could handle EMI channels if needed, but currently we do not handle them in this function.
        }
          
    }


    std::cout<< "entering UseGSCM()" << std::endl;
    if (GSCMunity->UseGSCM()){
      std::cout<< "GSCM bool: " << GSCMunity->UseGSCM() << " GSCM size: " << GSCMvec->size()<< std::endl;
        for(unsigned int l =0; l< GSCMvec->size(); l++){

            int ind = GSCMvec->size() - l - 1;
//          int ind = l;
            // std::cout<< "GSCM size " << GSCMvec->size()<< " index is: " << ind << std::endl;
            // std::cout<< "average RSS per 1MHz estimation GSCM for Tx: "<< GSCMvec->Get(ind)->Tx()<< " Rx: "<< GSCMvec->Get(ind)->Rx() << " from Unity to NS3 with: "<< GSCMvec->Get(ind)->rss() << " dBm/MHz" << std::endl;
            // std::cout<< "average RSS per 10MHz estimation GSCM for Tx: "<< GSCMvec->Get(ind)->Tx()<< " Rx: "<< GSCMvec->Get(ind)->Rx() << " from Unity to NS3 with: "<< GSCMvec->Get(ind)->rss() - 53 << " dBm/10MHz" << std::endl;

        }
    }


    for (unsigned int i = 0; i < posit->size() ; i++) {

        std::cout<< "Position size from Unity3D for gNB: " << posit->size() << std::endl;

        int veh = posit->Get(i)->id();
        if (i == 0)
        {
            
            // std::cout<< "Position update received from Unity3D for gNB: " << veh << " With NS3 ID: "<< gnbNodes.Get(0)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< std::endl;
            Ptr<ConstantPositionMobilityModel> mobil = gnbNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
            mobil->SetPosition(Vector(posit->Get(i)->x(), posit->Get(i)->z(), posit->Get(i)->y()));
            // std::cout<< "Gnb position from Unity is updated" << std::endl;
    //      std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< " EEB is set to: "<< EmergencyBrake->Get(EmergencyBrake->Get(i)->VehID())->triggered() << std::endl;
            /* code */
        }
        if (i==1)
        {
            Ptr<ConstantPositionMobilityModel> mobil = ueNodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
            mobil->SetPosition(Vector(posit->Get(i)->x(), posit->Get(i)->z(), posit->Get(i)->y()));
    //      std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< " EEB is set to: "<< EmergencyBrake->Get(EmergencyBrake->Get(i)->VehID())->triggered() << std::endl;
            // std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< ueNodes.Get(0)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< std::endl;/* code */
        }
        
        
    }

    std::cout<< "Unity3D update received. Unity3D time: " << info->Get(0)->timeStamp() << " NS3 current time: "<< Simulator::Now() << std::endl;

    // If Unity3D simulation is about to stop and we recieved termination message
    // if(termin){
    if(termin){
        std::cout<< "Command to terminate received from Unity3D. Stopping simulation and closing NS3 thread now!" << std::endl;
        // Simulator::ScheduleNow(&PrintStatistics);
        // Simulator::Schedule(Simulator::Now() - MilliSeconds(1), &PrintStatistics);
        APIconnect_pub->close();
        APIconnect_sub->close();
        
        Simulator::Stop();
    }
    else{
        // Simulator::ScheduleNow(&PrintStatistics);
        Simulator::Schedule (Seconds(FixedUpdInterval), &SetAllParametersFromApiWithInterferenceSwitch, gnbNodes, ueNodes, APIconnect, FixedUpdInterval, SpectrumModelPtr);
    }

    // Simulator::Schedule (Seconds(FixedUpdInterval), &SetAllParametersFromApiWithInterferenceSwitch, gnbNodes, ueNodes, APIconnect, FixedUpdInterval, SpectrumModelPtr);

}


// CustomInterferer class: injects a dummy PSD signal into the spectrum channel during DL transmission
class CustomInterferer {
public:
    CustomInterferer(Ptr<ns3::SpectrumChannel> channel)
        : m_channel(channel) {}

    // void InjectInterference(const ns3::GnbPhyPacketCountParameter& param) {
    void InjectInterference(ns3::GnbPhyPacketCountParameter param) {    
        if (!m_channel) {
            NS_LOG_ERROR("SpectrumChannel pointer is null in CustomInterferer!");
            return;
        }
        if (m_channel->GetNDevices() == 0) {
            NS_LOG_ERROR("No devices attached to SpectrumChannel!");
            return;
        }
        Ptr<NetDevice> dev = m_channel->GetDevice(1); // Assuming the second device is the UE
        // Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(dev);
        // if (!gnbDev) {
        //     NS_LOG_ERROR("Device is not a NrGnbNetDevice!");
        //     return;
        // }
        Ptr<NrUeNetDevice> ueDev = DynamicCast<NrUeNetDevice>(dev);
        if (!ueDev) {
            NS_LOG_ERROR("Device is not a NrGnbNetDevice!");
            return;
        }
        // Ptr<NrGnbPhy> gnbPhy = gnbDev->GetPhy(0);
        // Ptr<NrSpectrumPhy> dlSpectrumPhy = gnbPhy->GetSpectrumPhy();
        Ptr<NrUePhy> uePhy = ueDev->GetPhy(0);
        Ptr<NrSpectrumPhy> dlSpectrumPhy = uePhy->GetSpectrumPhy();
        Ptr<NrInterference> ueInterf = dlSpectrumPhy->GetNrInterference();
        if (!dlSpectrumPhy) {
            NS_LOG_ERROR("Downlink SpectrumPhy not found!");
            return;
        }
        Ptr<const ns3::SpectrumModel> model = dlSpectrumPhy->GetRxSpectrumModel();

        // ------------------------------------------------------------------
        // Build composite Noise PSD = Thermal(kT) * NF + External EMI PSD
        // ------------------------------------------------------------------
        // Assumptions:
        //  - EMI_psd entries (if provided) are pure external interference densities (W/Hz).
        //  - We want to include receiver thermal noise floor + specified NF (5 dB) once.
        //  - If EMI_psd is empty we still provide thermal+NF plus a tiny placeholder EMI.
        //  - Using SetNoisePowerSpectralDensity() folds EMI into stationary noise; use
        //    actual channel transmissions instead if you need time-varying interferers.

        const double nfDb = 5.0;                 // Target Noise Figure to apply (dB)
        const double kT_dBmPerHz = -174.0;       // Thermal noise density at 290K
        const double kT_W_Hz = std::pow(10.0, (kT_dBmPerHz - 30.0) / 10.0);
        const double nfLinear = std::pow(10.0, nfDb / 10.0);
        const double thermalPerHz = kT_W_Hz * nfLinear; // W/Hz

        Ptr<ns3::SpectrumValue> compositePsd = ns3::Create<ns3::SpectrumValue>(model);
        const double dummyEmiPsd = 1e-12; // very low external EMI placeholder (W/Hz)
        bool emiEmpty = EMI_psd.empty();

        double rbBandwidthHz = 0.0;
        if (model->GetNumBands() > 0) {
            rbBandwidthHz = model->Begin()->fh - model->Begin()->fl; // same for all RBs
        }
        double sumThermalW = 0.0;
        double sumEmiW = 0.0;
        double sumTotalW = 0.0;

        for (size_t i = 0; i < model->GetNumBands(); ++i) {
            double emi = emiEmpty ? dummyEmiPsd : (i < EMI_psd.size() ? EMI_psd[i] : dummyEmiPsd);
            double total = thermalPerHz + emi;
            (*compositePsd)[i] = total;
            sumThermalW += thermalPerHz * rbBandwidthHz;
            sumEmiW += emi * rbBandwidthHz;
            sumTotalW += total * rbBandwidthHz;
        }

        Ptr<ns3::SpectrumSignalParameters> interferenceParams = ns3::Create<ns3::SpectrumSignalParameters>();
        interferenceParams->psd = compositePsd;
        interferenceParams->duration = ns3::Seconds(0.002); // 2 ms duration (arbitrary here)
        interferenceParams->txPhy = dlSpectrumPhy;

        ueInterf->SetNoisePowerSpectralDensity(interferenceParams->psd);

        std::cout << "[Composite Noise PSD] Applied (NF=" << nfDb << " dB)." << std::endl;
        std::cout << "  Thermal base density: " << thermalPerHz << " W/Hz" << std::endl;
        if (emiEmpty) {
            std::cout << "  EMI_psd empty -> using dummy EMI value: " << dummyEmiPsd << " W/Hz per bin" << std::endl;
        }
        std::cout << "  Approx integrated thermal: " << WtodBmlocalContext(sumThermalW) << " dBm" << std::endl;
        std::cout << "  Approx integrated EMI:     " << WtodBmlocalContext(sumEmiW) << " dBm" << std::endl;
        std::cout << "  Approx integrated total:   " << WtodBmlocalContext(sumTotalW) << " dBm" << std::endl;
        std::cout << "  Integral(compositePsd):    " << WtodBmlocalContext(Integral(*interferenceParams->psd)) << " dBm" << std::endl;
        std::cout << "  (If you need dynamic/time-varying EMI, inject via channel transmissions instead.)" << std::endl;
    }
private:
    Ptr<ns3::SpectrumChannel> m_channel;
};


// Minimal EMI Chunk Processor for ns-3
class EmiChunkProcessor : public ns3::NrChunkProcessor {
public:
  EmiChunkProcessor(ns3::Ptr<const ns3::SpectrumModel> model)
    : m_spectrumModel(model) {}
  virtual ~EmiChunkProcessor() {}

  void Evaluate(ns3::Ptr<ns3::SpectrumValue> sinr, ns3::Time duration) {
    if (!m_spectrumModel || EMI_psd.empty()) return;
    ns3::Ptr<ns3::SpectrumValue> emiSv = ns3::Create<ns3::SpectrumValue>(m_spectrumModel);
    for (size_t i = 0; i < EMI_psd.size() && i < emiSv->GetSpectrumModel()->GetNumBands(); ++i) {
      (*emiSv)[i] = EMI_psd[i];
    }
    // Add EMI to the SINR calculation
    for (size_t i = 0; i < emiSv->GetSpectrumModel()->GetNumBands(); ++i) {
      (*sinr)[i] += (*emiSv)[i];
    }
  }
private:
  ns3::Ptr<const ns3::SpectrumModel> m_spectrumModel;
};


int
main(int argc, char* argv[])
{

    // LogComponentEnable("MultiModelSpectrumChannel", LOG_ALL);   
    // LogComponentEnable("NrUePhy", LOG_ALL);
    // LogComponentEnable("NrGnbPhy", LOG_ALL);
    // LogComponentEnable("NrAmc", LOG_ALL);
    //  LogComponentEnable("NrSpectrumPhy", LOG_ALL);
    //  LogComponentEnable("NrHelperSivert", LOG_ALL);
    //  LogComponentEnable("NrEesmErrorModel", LOG_ALL);
    //  LogComponentEnable("NrEesmIrT2", LOG_ALL);
    //  LogComponentEnable("NrInterference", LOG_ALL); 

    //  LogComponentEnable("CttcNrMimoDemoApi", LOG_ALL);
    //  LogComponentEnable("ThreeGppSpectrumPropagationLossModel", LOG_ALL); // Parent calss that also includes the beamforming and spectrum calculation.
    //  LogComponentEnable("SivertThreeGppSpectrumPropagationLossModel", LOG_ALL); // Parent calss that also includes the beamforming and spectrum calculation.
    //  LogComponentEnable("ThreeGppChannelModel", LOG_ALL); // actual implemetation of the channel 38.901 equation - ComplexChannelMatrix


    Config::SetDefault("ns3::NrHelperSivert::EnableMimoFeedback", BooleanValue(true));
    Config::SetDefault("ns3::NrPmSearch::SubbandSize", UintegerValue(16));
    bool useMimoPmiParams = false;

    NrHelperSivert::AntennaParams apUe;
    NrHelperSivert::AntennaParams apGnb;
    apUe.antennaElem = "ns3::ThreeGppAntennaModel";
    apUe.nAntCols = 2;
    apUe.nAntRows = 1;
    apUe.nHorizPorts = 2;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = false;
    apGnb.antennaElem = "ns3::ThreeGppAntennaModel";
    // 6 Ports, 2 rows, 3 columns
    // apGnb.nAntCols = 3;
    // apGnb.nAntRows = 1;
    // apGnb.nHorizPorts = 3;
    // apGnb.nVertPorts = 1;
    // apGnb.isDualPolarized = true;
    // 2 Ports, 1 row, 2 columns
    apGnb.nAntCols = 2;
    apGnb.nAntRows = 1;
    apGnb.nHorizPorts = 2;
    apGnb.nVertPorts = 1;
    apGnb.isDualPolarized = false;

    // The polarization slant angle in degrees in case of x-polarized
    double polSlantAngleGnb = 0.0;
    double polSlantAngleUe = 90.0;
    // The bearing angles in degrees
    double bearingAngleGnb = 0.0;
    double bearingAngleUe = 180.0;

    // Traffic parameters
    uint32_t udpPacketSize = 1000;
    // For 2x2 MIMO and NR MCS table 2, packet interval is 40000 ns to
    // reach 200 mb/s
    Time packetInterval = NanoSeconds(40000);
    Time udpAppStartTime = MilliSeconds(4);

    // Interference
    bool enableInterfNode = false; // if true an additional pair of gNB and UE will be created
                                   // to create an interference towards the original pair
    double interfDistance = 100.0; // the distance in meters between the original node pair, and the
                                   // interfering node pair
    double interfPolSlantDelta = 0; // the difference between the pol. slant angle between the
                                    // original node and the interfering one

    // Other simulation scenario parameters
    // Time simTime = MilliSeconds(1000);
    Time simTime = Seconds(1000);
    uint16_t gnbUeDistance = 20; // meters - replaced with input from unity
    uint16_t numerology = 0;
    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    double txPowerGnb = 0; // dBm
    double txPowerUe = 23;  // dBm
    uint16_t updatePeriodMs = 100;
    std::string errorModel = "ns3::NrEesmIrT2";
    // std::string scheduler = "ns3::NrMacSchedulerTdmaRR";
    // std::string scheduler = "ns3::NrMacSchedulerTdmaPF";
    std::string scheduler = "ns3::NrMacSchedulerOfdmaPF";
    std::string beamformingMethod = "ns3::DirectPathBeamforming";
    /**
     *   UMi_StreetCanyon,      //!< UMi_StreetCanyon
     *   UMi_StreetCanyon_LoS,  //!< UMi_StreetCanyon where all the nodes will be in Line-of-Sight
     *   UMi_StreetCanyon_nLoS, //!< UMi_StreetCanyon where all the nodes will not be in
     *
     */

    uint16_t losCondition = 0;

    // Where the example stores the output files.
    std::string simTag = "default";
    std::string outputDir = "./";
    bool logging = false;

    CommandLine cmd(__FILE__);
    /**
     * The main parameters for testing MIMO
     */
    cmd.AddValue("enableMimoFeedback", "ns3::NrHelperSivert::EnableMimoFeedback");
    cmd.AddValue("pmSearchMethod", "ns3::NrHelperSivert::PmSearchMethod");
    cmd.AddValue("fullSearchCb", "ns3::NrPmSearchFull::CodebookType");
    cmd.AddValue("rankLimit", "ns3::NrPmSearch::RankLimit");
    cmd.AddValue("subbandSize", "ns3::NrPmSearch::SubbandSize");
    cmd.AddValue("downsamplingTechnique", "ns3::NrPmSearch::DownsamplingTechnique");
    cmd.AddValue("numRowsGnb", "Number of antenna rows at the gNB", apGnb.nAntRows);
    cmd.AddValue("numRowsUe", "Number of antenna rows at the UE", apUe.nAntRows);
    cmd.AddValue("numColumnsGnb", "Number of antenna columns at the gNB", apGnb.nAntCols);
    cmd.AddValue("numColumnsUe", "Number of antenna columns at the UE", apUe.nAntCols);
    cmd.AddValue("numVPortsGnb",
                 "Number of vertical ports of the antenna at the gNB",
                 apGnb.nVertPorts);
    cmd.AddValue("numVPortsUe",
                 "Number of vertical ports of the antenna at the UE",
                 apUe.nVertPorts);
    cmd.AddValue("numHPortsGnb",
                 "Number of horizontal ports of the antenna the gNB",
                 apGnb.nHorizPorts);
    cmd.AddValue("numHPortsUe",
                 "Number of horizontal ports of the antenna at the UE",
                 apUe.nHorizPorts);
    cmd.AddValue("xPolGnb",
                 "Whether the gNB antenna array has the cross polarized antenna "
                 "elements.",
                 apGnb.isDualPolarized);
    cmd.AddValue("xPolUe",
                 "Whether the UE antenna array has the cross polarized antenna "
                 "elements.",
                 apUe.isDualPolarized);
    cmd.AddValue("polSlantAngleGnb",
                 "Polarization slant angle of gNB in degrees",
                 polSlantAngleGnb);
    cmd.AddValue("polSlantAngleUe", "Polarization slant angle of UE in degrees", polSlantAngleUe);
    cmd.AddValue("bearingAngleGnb", "Bearing angle of gNB in degrees", bearingAngleGnb);
    cmd.AddValue("bearingAngleUe", "Bearing angle of UE in degrees", bearingAngleUe);
    cmd.AddValue("enableInterfNode", "Whether to enable an interfering node", enableInterfNode);
    cmd.AddValue(
        "interfDistance",
        "The distance between the pairs of gNB and UE (the original and the interfering one)",
        interfDistance);
    cmd.AddValue("interfPolSlantDelta",
                 "The difference between the pol. slant angles of the original pairs of gNB and UE "
                 "and the interfering one",
                 interfPolSlantDelta);

    /**
     * Other simulation parameters
     */
    cmd.AddValue("packetSize",
                 "packet size in bytes to be used by best effort traffic",
                 udpPacketSize);
    cmd.AddValue("packetInterval", "Inter-packet interval for CBR traffic", packetInterval);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numerology", "The numerology to be used", numerology);
    cmd.AddValue("centralFrequency", "The system frequency to be used in band 1", centralFrequency);
    cmd.AddValue("bandwidth", "The system bandwidth to be used", bandwidth);
    cmd.AddValue("txPowerGnb", "gNB TX power", txPowerGnb);
    cmd.AddValue("txPowerUe", "UE TX power", txPowerUe);
    cmd.AddValue("gnbUeDistance",
                 "The distance between the gNB and the UE in the scenario",
                 gnbUeDistance);
    cmd.AddValue(
        "updatePeriodMs",
        "Channel update period in ms. If set to 0 then the channel update will be disabled",
        updatePeriodMs);
    cmd.AddValue("errorModel",
                 "Error model: ns3::NrEesmCcT1, ns3::NrEesmCcT2, "
                 "ns3::NrEesmIrT1, ns3::NrEesmIrT2, ns3::NrLteMiErrorModel",
                 errorModel);
    cmd.AddValue("scheduler",
                 "The scheduler: ns3::NrMacSchedulerTdmaRR, "
                 "ns3::NrMacSchedulerTdmaPF, ns3::NrMacSchedulerTdmaMR,"
                 "ns3::NrMacSchedulerTdmaQos, ns3::NrMacSchedulerOfdmaRR, "
                 "ns3::NrMacSchedulerOfdmaPF, ns3::NrMacSchedulerOfdmaMR,"
                 "ns3::NrMacSchedulerOfdmaQos",
                 scheduler);
    cmd.AddValue("beamformingMethod",
                 "The beamforming method: ns3::CellScanBeamforming,"
                 "ns3::CellScanBeamformingAzimuthZenith,"
                 "ns3::CellScanQuasiOmniBeamforming,"
                 "ns3::DirectPathBeamforming,"
                 "ns3::QuasiOmniDirectPathBeamforming,"
                 "ns3::DirectPathQuasiOmniBeamforming",
                 beamformingMethod);
    cmd.AddValue("losCondition",
                 "0 - for 3GPP channel condition model,"
                 "1 - for always LOS channel condition model,"
                 "2 - for always NLOS channel condition model",
                 losCondition);
    cmd.AddValue("simTag",
                 "tag to be appended to output filenames to distinguish simulation campaigns",
                 simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("useMimoPmiParams", "Configure via the MimoPmiParams structure", useMimoPmiParams);
    // Parse the command line
    cmd.Parse(argc, argv);

    // convert angle values into radians
    apUe.bearingAngle = bearingAngleUe * (M_PI / 180);
    apUe.polSlantAngle = polSlantAngleUe * (M_PI / 180);
    apGnb.bearingAngle = bearingAngleGnb * (M_PI / 180);
    apGnb.polSlantAngle = polSlantAngleGnb * (M_PI / 180);

    NS_ABORT_IF(centralFrequency < 0.5e9 && centralFrequency > 100e9);
    NS_ABORT_UNLESS(losCondition < 3);

    if (logging)
    {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("NrPdcp", LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(updatePeriodMs)));

    uint16_t pairsToCreate = 1;
    if (enableInterfNode)
    {
        pairsToCreate = 2;
    }

    NodeContainer gnbContainer;
    gnbContainer.Create(pairsToCreate);
    NodeContainer ueContainer;
    ueContainer.Create(pairsToCreate);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 10.0));
    positionAlloc->Add(Vector(gnbUeDistance, 0.0, 1.5));
    // the positions for the second interfering pair of gNB and UE
    if (enableInterfNode)
    {
        positionAlloc->Add(Vector(0.0, interfDistance, 10.0));
        positionAlloc->Add(Vector(gnbUeDistance, interfDistance, 1.5));
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gnbContainer.Get(0));
    mobility.Install(ueContainer.Get(0));
    // install mobility of the second pair of gNB and UE
    if (enableInterfNode)
    {
        mobility.Install(gnbContainer.Get(1));
        mobility.Install(ueContainer.Get(1));
    }

    /** 
     * Create a SIVERT API connection to receive updates from Unity3D.
     */

    zmq::socket_t initSocket (context, ZMQ_REP);
    std::cout << "Connecting to Unity3D socket…" << std::endl;


    initSocket.bind ("tcp://127.0.0.1:5555");
    double UnityFixedUpd; // FixedUpdate step at which Unity3D updates Physics (our API update rate)
    double SimStart; // When we want to start simulating communication nodes
    int NumVeh; // Nodes expected in the Unity3D simulation
    bool UseGSCM; // GSCM channel model selector
    
    double InterferenceLevel_dBm;


    while (true){
        std::string updStep = s_recv (initSocket);
        std::string SimDelay = s_recv (initSocket);
        std::string VehNum = s_recv(initSocket);
        std::string ChModSel = s_recv(initSocket);
        std::string DisbaleEMIforDemo = s_recv(initSocket);
        std::string InterferenceLevel = s_recv(initSocket);

        if (ChModSel.compare("True") == 0){
            UseGSCM = true;
        } else
        {
            UseGSCM = false;
        }

        UnityFixedUpd = ::atof(updStep.c_str()); // converting string received back into double FixedUpdate
        SimStart = ::atof(SimDelay.c_str());
        NumVeh = ::atof(VehNum.c_str());
        InterferenceLevel_dBm = ::atof(InterferenceLevel.c_str());
        
        if (DisbaleEMIforDemo.compare("True") == 0){
            IsEMIdisabled = true;
        } else
        {
            IsEMIdisabled = false;
        }

        if (NumVeh>0){
            break;
        }
    }
    std::cout << "Received Hello Message from Unity3D with FixedUpd value: " << UnityFixedUpd << std::endl;
    std::cout << "Simulation will start at: " << SimStart << std::endl;
    std::cout << "Number of nodes received: " << NumVeh << std::endl;
    std::cout << "Interference level received: " << InterferenceLevel_dBm << std::endl;
    double InterferenceLevel_dBW = InterferenceLevel_dBm - 30; // Substract 30 to convert from dBW to dBm (since the interference in the spectrum channel is in dBW)

    std::string rplMSG = "HiUnity";
    s_send(initSocket, rplMSG);

    std::cout<<"NS3 process and API is running!"<<std::endl;
    std::cout<<"C-ITS GSCM 5G LENA scenario is chosen"<<std::endl;

    /**
     * Create the NR helpers that will be used to create and setup NR devices, spectrum, ...
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    // Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    Ptr<NrHelperSivert> nrHelper = CreateObject<NrHelperSivert>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    /**
     * Prepare spectrum. Prepare one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by the input parameters.
     *
     *
     * The configured spectrum division is:
     * ------------Band--------------
     * ------------CC1----------------
     * ------------BWP1---------------
     */

    BandwidthPartInfo::Scenario scenario =
        BandwidthPartInfo::Scenario(BandwidthPartInfo::UMi_StreetCanyon + losCondition);

    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,
                                                   bandwidth,
                                                   numCcPerBand,
                                                   scenario);
    OperationBandInfo bandTDD = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    /**
     * Configure NrHelper, prepare most of the parameters that will be used in the simulation.
     */
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                TimeValue(MilliSeconds(updatePeriodMs)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scheduler));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(TypeId::LookupByName(beamformingMethod)));
    // Core latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // We can configure not only via Configure::SetDefault, but also via the MimoPmiParams structure
    if (useMimoPmiParams)
    {
        ns3::NrHelperSivert::MimoPmiParams params;
        // Use a standards-compliant subband size for 20 MHz @ mu=0 (106 PRBs total):
        // Allowed values are {8, 16} as per TS 38.214 and ns-3 checks
        params.subbandSize = 16;
        // params.subbandSize = 106; // for 20 MHz bandwidth TDMA mode
        params.fullSearchCb = "ns3::NrCbTypeOneSp";
        params.pmSearchMethod = "ns3::NrPmSearchFull";
        // Smoother subband downsampling to reduce PMI volatility
        params.downsamplingTechnique = "AveragePRB";
        params.rankLimit = 2;
        nrHelper->SetupMimoPmi(params);
    }

    /**
     * Configure gNb antenna
     */
    nrHelper->SetupGnbAntennas(apGnb);
    /**
     * Configure UE antenna
     */
    nrHelper->SetupUeAntennas(apUe);

    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPowerGnb));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPowerUe));

    uint32_t bwpId = 0;
    // gNb routing between bearer type and bandwidth part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));
    // UE routing between bearer type and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpId));

    /**
     * Initialize channel and pathloss, plus other things inside band.
     */
    // nrHelper->InitializeOperationBand(&band);
    // BandwidthPartInfoPtrVector allBwps;
    // allBwps = CcBwpCreator::GetAllBwps({band});

    /**
     * SIVERT helper initialisation
    */
    nrHelper->SivertInitializeOperationBand(&bandTDD);
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({bandTDD});

    

    // getting the pointer to the spectrum channel model for the Sivert channel access
    Ptr<PhasedArraySpectrumPropagationLossModel> fading =
    DynamicCast<PhasedArraySpectrumPropagationLossModel>(allBwps.at(0).get()->m_channel->GetPhasedArraySpectrumPropagationLossModel());
    Ptr<SivertThreeGppSpectrumPropagationLossModel> ChModelPointer =
    DynamicCast<SivertThreeGppSpectrumPropagationLossModel>(fading);
    ///////////////////
    NS_LOG_UNCOND("m_channel: " << allBwps.at(0).get()->m_channel);
    NS_LOG_UNCOND("GetPropagationLossModel: " << allBwps.at(0).get()->m_channel->GetPropagationLossModel());
    NS_LOG_DEBUG("Initializing the SivertThreeGppSpectrumPropagationLossModel with dummy values.");
    // double init_gain = 1e-13; // poor connectivity - BLER is high Demo
    double init_gain = 8e-30; // bad connectivity - BLER appx 1 for Demo
    // double init_gain = 5e-13; // fine connectivity
    // double init_gain = 1e-11; // good connectivity
    // double init_gain = 1e-8; // great connectivity
    int num_of_sub_bands = 300;
    int num_nodes = 2; // 1 gNB and 1 UE
    std::vector<double> gscmSpectrumGain;
    std::complex<double> dummyComplexValue(0.000001, 0.000001);
    size_t numSymbols = 200; // or whatever size you need
    for (size_t i = 0; i < num_of_sub_bands; i++) {
        // std::cout << "PSD Gain Coefficient[" << i << "]: " << psdGains->Get(i) << std::endl;
        int r = GetRandomIntInRange(1, 2000);  // random integer between 1 and 10 (inclusive)
        gscmSpectrumGain.push_back(init_gain * r);
    }  
    for (size_t i = 0; i < num_nodes + 10; i++)
    {
        for (size_t j = 0; j < num_nodes + 10; j++) // fililng up the channel with dummy vaues to avoid runtime error if the unity channel is delayed
        {
            ChModelPointer->SetChannelFromUnity3D(i, j, gscmSpectrumGain);
            ChModelPointer->SetComplexChannelFromUnity3D(i, j, std::vector<std::complex<double>>(numSymbols, dummyComplexValue));
        }

    }
    NS_LOG_DEBUG("Sample gain for [0][1]: " << ChModelPointer->GetChannelVectorFromUnity3D(0, 1)[0] << std::endl);
    /**
     * Finally, create the gNB and the UE device.
     */
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);

    if (enableInterfNode && interfPolSlantDelta != 0)
    {
        // reconfigure the polarization slant angle of the interferer
        nrHelper->GetGnbPhy(gnbNetDev.Get(1), 0)
            ->GetSpectrumPhy()
            ->GetAntenna()
            ->SetAttribute("PolSlantAngle",
                           DoubleValue((polSlantAngleGnb + interfPolSlantDelta) * (M_PI / 180)));
        nrHelper->GetUePhy(ueNetDev.Get(1), 0)
            ->GetSpectrumPhy()
            ->GetAntenna()
            ->SetAttribute("PolSlantAngle",
                           DoubleValue((polSlantAngleUe + interfPolSlantDelta) * (M_PI / 180)));
    }

    /**
     * Fix the random stream throughout the nr, propagation, and spectrum
     * modules classes. This configuration is extremely important for the
     * reproducibility of the results.
     */
    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // When all the configuration is done, explicitly call UpdateConfig ()
    // TODO: Check if this is necessary to call when we do not reconfigure anything after devices
    // have been created
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }
    

    // create the Internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueContainer);
    Ipv4InterfaceContainer ueIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    // Set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueContainer.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

    // attach each UE to its gNB according to desired scenario
    nrHelper->AttachToGnb(ueNetDev.Get(0), gnbNetDev.Get(0));
    if (enableInterfNode)
    {
        nrHelper->AttachToGnb(ueNetDev.Get(1), gnbNetDev.Get(1));
    }

    /**
     * Install DL traffic part.
     */
    uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    // The sink will always listen to the specified ports
    UdpServerHelper dlPacketSink(dlPort);
    // The server, that is the application which is listening, is installed in the UE
    serverApps.Add(dlPacketSink.Install(ueContainer));
    /**
     * Configure attributes for the CBR traffic generator, using user-provided
     * parameters
     */
    UdpClientHelper dlClient;
    dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
    dlClient.SetAttribute("Interval", TimeValue(packetInterval));

    // The bearer that will carry the traffic
    NrEpsBearer epsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);

    // The filter for the traffic
    Ptr<NrEpcTft> dlTft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlPktFilter;
    dlPktFilter.localPortStart = dlPort;
    dlPktFilter.localPortEnd = dlPort;
    dlTft->Add(dlPktFilter);

    /**
     * Let's install the applications!
     */
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < ueContainer.GetN(); ++i)
    {
        Ptr<Node> ue = ueContainer.Get(i);
        Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
        Address ueAddress = ueIpIface.GetAddress(i);

        // The client, who is transmitting, is installed in the remote host,
        // with destination address set to the address of the UE
        dlClient.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        clientApps.Add(dlClient.Install(remoteHost));

        // Activate a dedicated bearer for the traffic
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, epsBearer, dlTft);
    }

    // start UDP server and client apps
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(ueContainer);

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));


    APIconnect_sub = s_sub_socket (context, "tcp://127.0.0.1:8002");
    APIconnect_pub = s_pub_socket (context, "tcp://127.0.0.1:8001");
    Simulator::ScheduleNow(
      &SetAllParametersFromApiWithInterferenceSwitch,
      gnbContainer,
      ueContainer,
      APIconnect_sub,
      UnityFixedUpd,
      ChModelPointer);



    // attaching EMI callback to the spectrum channel
    Ptr<SpectrumChannel> m_spectrumChannelPointer = allBwps.at(0).get()->m_channel;
    // m_spectrumChannelPointer->TraceConnectWithoutContext("TxPacketTraceGnb", MakeCallback(&DlTxInterceptionCallbackForEmi));

    // After gNB and channel setup, create the interferer
    double centerFreq = allBwps.at(0).get()->m_centralFrequency; // Set to your scenario's center frequency
    double bandwidthEMI = allBwps.at(0).get()->m_channelBandwidth; // Set to your scenario's bandwidth
 
    /*
    For EMI injection we need to monitor all the DL transmission (gNB to UE) and inject EMI PSD from Unity3D
    upon overhearing the DL transmission. That is to simulate the EMI levels produced by the UE surroundings (measured in the EMC lab)
    TxPacketTraceGnb tracesource at the gNB triggered upon new DL transmission. We create a custom interferer class that uses as input
    UE spectrum channel and EMI levels from Unity3D. Interference is injected directly to the NrInterference for each DL transmission.
    */

    if (!IsEMIdisabled)
    {
        Ptr<SpectrumChannel> ueSpectrumChannel = nrHelper->GetUePhy(ueNetDev.Get(0), 0)->GetSpectrumPhy()->GetSpectrumChannel();
        CustomInterferer interferer(ueSpectrumChannel);

        Ptr<ns3::NrSpectrumPhy> gnbPhy = nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->GetSpectrumPhy();
        gnbPhy->TraceConnectWithoutContext("TxPacketTraceGnb", ns3::MakeCallback(&CustomInterferer::InjectInterference, &interferer));
    }
    


    Simulator::Stop(simTime);
    Simulator::Run();

    // Print per-flow statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    std::string filename = outputDir + "/" + simTag;


    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    std::ofstream outFile;
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf(std::ios_base::fixed);

    double actualSimTime = Simulator::Now().GetSeconds();
    double flowDuration = actualSimTime - udpAppStartTime.GetSeconds();

    // double flowDuration = (simTime - udpAppStartTime).GetSeconds();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t)t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                << t.destinationAddress << ":" << t.destinationPort << ") proto "
                << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0
                << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            // Measure the duration of the flow from receiver's perspective
            averageFlowThroughput += i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;

            outFile << "  Throughput: " << i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000
                    << " Mbps\n";
            
            double per = 1.0 - static_cast<double>(i->second.rxPackets) / i->second.txPackets;
            outFile << "  Packet Error Rate (PER): " << per << "\n";

            outFile << "  Mean delay:  "
                    << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms\n";
            // outFile << "  Mean upt:  " << i->second.uptSum / i->second.rxPackets / 1000/1000 << "
            // Mbps \n";
            outFile << "  Mean jitter:  "
                    << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
            
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Packet Error Rate (PER): N/A (0 Tx)\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    outFile << "\n\n  Mean flow throughput: " << averageFlowThroughput / stats.size() << "\n";
    outFile << "  Mean flow delay: " << averageFlowDelay / stats.size() << "\n";
    outFile.close();
    std::ifstream f(filename.c_str());
    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    Simulator::Destroy();
    return 0;
}


