use crate::{
    dev::{dev10g_init_sfi, dev1g_init_sgmii},
    serdes10g, serdes1g, serdes6g,
    spi::Vsc7448Spi,
    VscError,
};
use userlib::*;
use vsc7448_pac::Vsc7448;

pub struct Bsp<'a> {
    vsc7448: &'a Vsc7448Spi,
}

impl<'a> Bsp<'a> {
    /// Constructs and initializes a new BSP handle
    pub fn new(vsc7448: &'a Vsc7448Spi) -> Result<Self, VscError> {
        let out = Bsp { vsc7448 };
        out.init()?;
        Ok(out)
    }

    pub fn init(&self) -> Result<(), VscError> {
        // See RFD144 for a detailed look at the design

        // Cubbies 0 through 7
        let serdes1g_cfg_sgmii = serdes1g::Config::new(serdes1g::Mode::Sgmii);
        for dev in 0..=7 {
            dev1g_init_sgmii(dev, &self.vsc7448)?;
            serdes1g_cfg_sgmii.apply(dev + 1, &self.vsc7448)?;
            // DEV1G[dev], SERDES1G[dev + 1], S[port + 1], SGMII
        }
        // Cubbies 8 through 21
        let serdes6g_cfg_sgmii = serdes6g::Config::new(serdes6g::Mode::Sgmii);
        for dev in 0..=13 {
            serdes6g_cfg_sgmii.apply(dev, &self.vsc7448)?;
            // DEV2G5[dev], SERDES6G[dev], S[port + 1], SGMII
        }
        // Cubbies 22 through 29
        for dev in 16..=23 {
            serdes6g_cfg_sgmii.apply(dev, &self.vsc7448)?;
            // DEV2G5[dev], SERDES6G[dev], S[port + 1], SGMII
        }
        // Cubbies 30 and 31
        let serdes10g_cfg_sgmii =
            serdes10g::Config::new(serdes10g::Mode::Sgmii)?;
        for dev in [27, 28] {
            serdes10g_cfg_sgmii.apply(dev - 25, &self.vsc7448)?;
            // DEV2G5[dev], SERDES10G[dev - 25], S[dev + 8], SGMII
        }

        ////////////////////////////////////////////////////////////////////////
        // PSC0/1, Technician 0/1, a few unused ports
        // These go over 2x QSGMII links:
        // - Ports 16-19 go through SERDES6G_14 to an on-board VSC8504 PHY
        //   (PHY4, U40), which is configured over MIIM from the SP
        // - Ports 20-23 go through SERDES6G_15 to the front panel board

        // Let's configure the on-board PHY first
        // Relevant pins are
        // - MIIM_SP_TO_PHY_MDC_2V5
        // - MIIM_SP_TO_PHY_MDIO_2V5
        // - MIIM_SP_TO_PHY_MDINT_2V5_L
        // - SP_TO_PHY4_COMA_MODE_2V5
        // - SP_TO_PHY4_RESET_2V5_L
        //
        // The PHY talks on MIIM addresses 0x4-0x7 (configured by resistors
        // on the board)
        // TODO

        // Now that the PHY is configured, we can bring up the VSC7448.  This
        // is very similar to how we bring up QSGMII in the dev kit BSP
        // (bsp/gemini_bu.rs)
        self.vsc7448
            .modify(Vsc7448::HSIO().HW_CFGSTAT().HW_CFG(), |r| {
                // Enable QSGMII mode for DEV1G_16-23 via SerDes6G_14/15
                let ena = r.qsgmii_ena();
                r.set_qsgmii_ena(ena | (1 << 10) | (1 << 11));
            })?;
        for dev in 16..=23 {
            // Reset the PCS TX clock domain.  In the SDK, this is accompanied
            // by the cryptic comment "BZ23738", which may refer to an errata
            // of some kind?
            self.vsc7448.modify(
                Vsc7448::DEV1G(dev).DEV_CFG_STATUS().DEV_RST_CTRL(),
                |r| {
                    r.set_pcs_tx_rst(0);
                },
            )?;
        }
        let serdes6g_cfg_qsgmii = serdes6g::Config::new(serdes6g::Mode::Qsgmii);
        serdes6g_cfg_qsgmii.apply(14, &self.vsc7448)?;
        serdes6g_cfg_qsgmii.apply(15, &self.vsc7448)?;
        for dev in 16..=23 {
            dev1g_init_sgmii(dev, &self.vsc7448)?;
        }

        ////////////////////////////////////////////////////////////////////////
        // DEV2G5[24], SERDES1G[0], S0, SGMII to Local SP
        serdes1g_cfg_sgmii.apply(0, &self.vsc7448)?;
        dev1g_init_sgmii(24, &self.vsc7448)?;

        ////////////////////////////////////////////////////////////////////////
        // DEV10G[0], SERDES10G[0], S33, SFI to Tofino 2
        let serdes_cfg = serdes10g::Config::new(serdes10g::Mode::Lan10g)?;
        dev10g_init_sfi(0, &serdes_cfg, &self.vsc7448)?;

        unimplemented!()
    }

    pub fn run(&self) -> ! {
        loop {
            hl::sleep_for(100);
        }
    }
}